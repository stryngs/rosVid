import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
__all__ = ['RTSPMediaFactory', 'RTSPServer', 'GLib']

Gst.init(None)

class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, args):
        super().__init__()

        cChain = self.codecChain(args)

        if not args.mipi:
            self.set_launch(f'latency=0 drop-on-latency=true ! '
                            f'v4l2src device={args.device} do-timestamp=true ! '
                            f'video/x-raw,format={args.format},'
                            f'width={args.width},height={args.height},'
                            f'framerate={args.fps}/1 ! '
                            f'videoconvert ! '
                            f'{cChain}')
        else:
            self.set_launch(f'libcamerasrc ! '
                            f'video/x-raw,'
                            f'width={args.width},height={args.height},'
                            f'framerate={args.fps}/1 ! '
                            f'videoconvert ! '
                            f'{cChain}')


    def codecChain(self, args):
        """Currently handlyes h264 or h265"""
        if args.codec == 'h264':
            if not args.mipi:
                return (f'x264enc tune=zerolatency speed-preset=ultrafast '
                        f'sliced-threads=true threads=4 '
                        f'key-int-max=30 '
                        f'bitrate={args.bitrate} ! '
                        f'h264parse ! '
                        f'rtph264pay config-interval=1 name=pay0 pt=96')
            return (f'x264enc tune=zerolatency speed-preset=ultrafast '
                    f'key-int-max=30 '
                    f'bitrate={args.bitrate} ! '
                    f'h264parse ! '
                    f'rtph264pay config-interval=1 name=pay0 pt=96')

        elif args.codec == 'h265':
            if not args.mipi:
                return (f'x265enc tune=zerolatency '
                        f'key-int-max=30 '
                        f'bitrate={args.bitrate} ! '
                        f'h265parse ! '
                        f'rtph265pay config-interval=1 name=pay0 pt=96')
            return (f'x265enc tune=zerolatency '
                    f'key-int-max=30 '
                    f'bitrate={args.bitrate} ! '
                    f'h265parse ! '
                    f'rtph265pay config-interval=1 name=pay0 pt=96')



class RTSPServer:
    """Serve the RTSP"""
    def __init__(self, args):
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(f'{args.port}')
        f = RTSPMediaFactory(args)
        self.server.get_mount_points().add_factory(f'/{args.endpoint}', f)
        self.server.attach(None)
