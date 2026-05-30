import threading

from libStreamer.lib.ros import TopicFrameBridge
__all__ = ['RTSPMediaFactory', 'RTSPServer', 'GLib']

print(f'[TRACE rtsp] imported {__file__}', flush = True)

Gst = None
GstRtspServer = None
_GLib = None
_FactoryClass = None


class _LazyGLib:
    def __getattr__(self, name):
        ensureGst()
        return getattr(_GLib, name)


GLib = _LazyGLib()


def ensureGst():
    """Import and initialize GStreamer after ROS topic setup has had a chance to run."""
    global Gst, GstRtspServer, _GLib
    if Gst is not None:
        print('[TRACE rtsp] ensureGst already initialized', flush = True)
        return

    print('[TRACE rtsp] ensureGst importing gi/Gst/GstRtspServer', flush = True)
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst as GstModule
    from gi.repository import GstRtspServer as GstRtspServerModule
    from gi.repository import GLib as GLibModule

    Gst = GstModule
    GstRtspServer = GstRtspServerModule
    _GLib = GLibModule
    Gst.init(None)
    print('[TRACE rtsp] ensureGst complete', flush = True)


def mediaFactoryClass():
    """Create the real GStreamer RTSP factory class after GstRtspServer is loaded."""
    global _FactoryClass
    ensureGst()
    if _FactoryClass is None:
        print('[TRACE rtsp] creating RTSPMediaFactory class', flush = True)
        class Factory(_RTSPMediaFactoryMixin, GstRtspServer.RTSPMediaFactory):
            pass

        _FactoryClass = Factory
    else:
        print('[TRACE rtsp] reusing RTSPMediaFactory class', flush = True)
    return _FactoryClass


def RTSPMediaFactory(args, bridge = None):
    """Return a configured GStreamer RTSP media factory."""
    factoryClass = mediaFactoryClass()
    return factoryClass(args, bridge)


class _RTSPMediaFactoryMixin:
    def __init__(self, args, bridge = None):
        super().__init__()
        self.set_shared(True)
        self.args = args
        self.bridge = bridge
        self.spinThread = None
        self.didTracePush = False
        self.timestamp = 0
        self.duration = Gst.SECOND // max(int(args.fps), 1)

        capture = getattr(args, 'capture', 'device')
        print((f'[TRACE rtsp] factory args capture={capture!r} '
               f'topic={getattr(args, "topic", None)!r} '
               f'device={getattr(args, "device", None)!r} '
               f'mipi={getattr(args, "mipi", None)!r} '
               f'width={getattr(args, "width", None)!r} '
               f'height={getattr(args, "height", None)!r} '
               f'fps={getattr(args, "fps", None)!r} '
               f'bridge={bridge is not None}'), flush = True)

        cChain = self.codecChain(args)
        aChain = self.audioChain(args) if getattr(args, 'audio', False) else ''

        if capture == 'topic':
            print('[TRACE rtsp] factory selected topic/appsrc branch', flush = True)
            self.topicLaunch(args, aChain)
        elif not args.mipi:
            launch = (f'v4l2src device={args.device} do-timestamp=true ! '
                      f'video/x-raw,'
                      f'width={args.width},height={args.height},'
                      f'framerate={args.fps}/1 ! '
                      f'videoconvert ! '
                      f'video/x-raw,format={args.format} ! '
                      f'{cChain} '
                      f'{aChain}')
            print(f'[TRACE rtsp] factory selected device/v4l2 branch launch={launch}', flush = True)
            self.set_launch(launch)
        else:
            launch = (f'libcamerasrc ! '
                      f'video/x-raw,'
                      f'width={args.width},height={args.height},'
                      f'framerate={args.fps}/1 ! '
                      f'videoconvert ! '
                      f'video/x-raw,format={args.format} ! '
                      f'{cChain} '
                      f'{aChain}')
            print(f'[TRACE rtsp] factory selected mipi/libcamera branch launch={launch}', flush = True)
            self.set_launch(launch)


    def topicLaunch(self, args, aChain):
        """Build the RTSP pipeline for ROS topic frame input."""
        if self.bridge is None:
            raise RuntimeError('[!] topic capture requires a ROS topic bridge')
        cChain = self.topicCodecChain(args)
        launch = (f'appsrc name=source is-live=true block=false format=time '
                  f'do-timestamp=false max-buffers=1 leaky-type=downstream ! '
                  f'video/x-raw,format=BGR,width={args.width},height={args.height},'
                  f'framerate={args.fps}/1 ! '
                  f'queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! '
                  f'videoconvert ! '
                  f'video/x-raw,format={args.format} ! '
                  f'{cChain} '
                  f'{aChain}')
        print(f'[TRACE rtsp] topicLaunch topic={args.topic!r} launch={launch}', flush = True)
        self.set_launch(launch)
        self.connect('media-configure', self.mediaConfigure)


    def mediaConfigure(self, factory, media):
        """Start pushing the latest topic frame into appsrc at the requested FPS."""
        element = media.get_element()
        appsrc = element.get_child_by_name('source')
        print(f'[TRACE rtsp] mediaConfigure appsrc_found={appsrc is not None}', flush = True)
        appsrc.set_property('emit-signals', False)
        GLib.timeout_add(max(1, int(1000 / max(int(self.args.fps), 1))),
                         self.pushFrame,
                         appsrc)


    def pushFrame(self, appsrc):
        """Push one frame into the RTSP appsrc."""
        frame = self.bridge.latestFrame()
        data = frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.pts = self.timestamp
        buf.dts = self.timestamp
        buf.duration = self.duration
        self.timestamp += self.duration
        ret = appsrc.emit('push-buffer', buf)
        if not self.didTracePush:
            print((f'[TRACE rtsp] first pushFrame bytes={len(data)} '
                   f'shape={getattr(frame, "shape", None)} ret={ret}'), flush = True)
            self.didTracePush = True
        elif ret != Gst.FlowReturn.OK:
            print(f'[TRACE rtsp] pushFrame ret={ret}', flush = True)
        return ret == Gst.FlowReturn.OK


    def codecChain(self, args):
        """Currently handles h264 or h265"""
        if args.codec == 'h264':
            if not args.mipi:
                return (f'x264enc tune=zerolatency speed-preset=ultrafast '
                        f'sliced-threads=true threads=4 '
                        f'key-int-max=30 '
                        f'bitrate={args.bitrate} ! '
                        f'{args.codec}parse ! '
                        f'rtp{args.codec}pay config-interval=1 name=pay0 pt=96')
            return (f'x264enc tune=zerolatency speed-preset=ultrafast '
                    f'key-int-max=30 '
                    f'bitrate={args.bitrate} ! '
                    f'{args.codec}parse ! '
                    f'rtp{args.codec}pay config-interval=1 name=pay0 pt=96')

        elif args.codec == 'h265':
            if not args.mipi:
                return (f'x265enc tune=zerolatency '
                        f'key-int-max=30 '
                        f'bitrate={args.bitrate} ! '
                        f'{args.codec}parse ! '
                        f'rtp{args.codec}pay config-interval=1 name=pay0 pt=96')
            return (f'x265enc tune=zerolatency '
                    f'key-int-max=30 '
                    f'bitrate={args.bitrate} ! '
                    f'{args.codec}parse ! '
                    f'rtp{args.codec}pay config-interval=1 name=pay0 pt=96')


    def topicCodecChain(self, args):
        """Build a low-latency encoder chain for appsrc topic frames."""
        if args.codec == 'h264':
            return (f'x264enc tune=zerolatency speed-preset=ultrafast '
                    f'sliced-threads=true threads=4 '
                    f'key-int-max=30 '
                    f'byte-stream=true aud=true bframes=0 '
                    f'bitrate={args.bitrate} ! '
                    f'video/x-h264,profile=baseline,stream-format=byte-stream,alignment=au ! '
                    f'h264parse config-interval=1 ! '
                    f'rtph264pay config-interval=1 name=pay0 pt=96')

        elif args.codec == 'h265':
            return (f'x265enc tune=zerolatency '
                    f'key-int-max=30 '
                    f'bitrate={args.bitrate} ! '
                    f'h265parse ! '
                    f'rtph265pay config-interval=1 name=pay0 pt=96')


    def audioChain(self, args):
        return (f'alsasrc device=hw:{args.acard},{args.adevice} ! '
                f'audioconvert ! audioresample ! '
                f'audio/x-raw,rate={args.arate},channels={args.achannels} ! '
                f'opusenc bitrate={args.abitrate} ! '
                f'rtpopuspay name=pay1 pt=97')



class RTSPServer:
    """Serve the RTSP"""
    def __init__(self, args):
        self.bridge = None
        self.spinThread = None
        capture = getattr(args, 'capture', 'device')
        print((f'[TRACE rtsp] server args capture={capture!r} '
               f'topic={getattr(args, "topic", None)!r} '
               f'device={getattr(args, "device", None)!r} '
               f'port={getattr(args, "port", None)!r} '
               f'endpoint={getattr(args, "endpoint", None)!r}'), flush = True)
        if capture == 'topic':
            print('[TRACE rtsp] server creating topic bridge', flush = True)
            self.bridge, self.spinThread = self.topicBridge(args)
        else:
            print('[TRACE rtsp] server not creating topic bridge', flush = True)

        ensureGst()
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(f'{args.port}')
        f = RTSPMediaFactory(args, self.bridge)
        self.server.get_mount_points().add_factory(f'/{args.endpoint}', f)
        attachId = self.server.attach(None)
        print(f'[TRACE rtsp] server attach_id={attachId} mount=/{args.endpoint} port={args.port}', flush = True)
        if attachId == 0:
            print('[TRACE rtsp] server attach failed; another server may already own the port', flush = True)


    def topicBridge(self, args):
        """Initialize ROS before GStreamer and start the topic frame bridge."""
        import rclpy

        if not rclpy.ok():
            rclpy.init(args = None)

        topicType = getattr(args, 'type', 'UInt8MultiArray')
        frameKey = getattr(args, 'key', 'frame')
        print(f'[TRACE rtsp] topicBridge subscribing topic={args.topic!r} type={topicType!r} key={frameKey!r}', flush = True)
        bridge = TopicFrameBridge(args.topic, args.width, args.height, topicType, frameKey)
        spinThread = threading.Thread(daemon = True, target = self.spinBridge, args = (rclpy, bridge))
        spinThread.start()
        print('[TRACE rtsp] topicBridge spin thread started', flush = True)
        return bridge, spinThread


    def spinBridge(self, rclpy, bridge):
        """Spin the ROS bridge without printing noisy traces during shutdown."""
        try:
            rclpy.spin(bridge)
        except Exception as exc:
            if exc.__class__.__name__ != 'ExternalShutdownException':
                raise
