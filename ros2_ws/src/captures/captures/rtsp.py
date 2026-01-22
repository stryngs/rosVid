#!/usr/bin/env python3
 
import cv2
import gi
import rclpy
import threading
import time
import pickle
import numpy as np
import yaml
from cv_bridge import CvBridge
from libStreamer.frameworks.gstreamer import ChainDecoder
from mavPool import Pool
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String, UInt8MultiArray
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class Rtsp:
    """Handler for capturing an RTSP stream"""
    def __init__(self, node: Node | None = None, args = None):
        """
        If the node is provided it will attach and share with that node.
        If the node is not provided this class will create its own node.
        """
        self.args = args or {}
        self.node = node
        self.thisNode = node is None

        ## QOS
        ### Tune this for individual mods
        self.qos = QoSProfile(depth = 10,
                              history = HistoryPolicy.KEEP_LAST,
                              reliability = ReliabilityPolicy.BEST_EFFORT)

        ## Decide if this node
        if self.thisNode:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node('rtsp')

        ## Param/Topic handlers
        self.paramHandler()
        self.topicHandler()

        ## Interface setup
        self.cMsg = CompressedImage()
        self.cMsg.format = 'jpeg'
        self.pMsg = UInt8MultiArray()

        ## libStreamer integration
        ### Need to use Gst.ElementFactory.find(<enc>)
        ### avdec_h264, nvh264dec, omxh264dec, vaapih264dec and v4l2h264dec
        self.cEncode = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpegQuality]

        self.chainDecoder = ChainDecoder(decoder = f'avdec_{self.node.get_parameter('codec').value}',
                                         format = 'BGR',
                                         height = self.height,
                                         width = self.width)

        ## Camera integration
        self.bridge = CvBridge()

        self.flag_Shutdown = threading.Event()
        self.lock_Snapshot = threading.Lock()
        self.lock_Thread = threading.Lock()

        self.new_Snapshot = None
        self.new_Timestamp = None
        
        self.loop_GLib = None
        self.pipeline_Gstreamer = None
        self.thread_Gstreamer = None
        self.sink_App = None

        ## Init GStreamer
        ### Debate on pulling this into libStreamer on the ChainDecoder side of the house
        try:
            Gst.init(None)
        except Exception as E:
            self.node.get_logger().error(f'[!] GStreamer {E}')

        ## GStreamer Pipeline
        self.thread_Gstreamer = threading.Thread(daemon = True, target = self.rtspThread)
        self.thread_Gstreamer.start()

        ## Callbacks
        self.timer = self.node.create_timer(self.pubSpeed_period, self.cback_Pub)

        ## mavPool connection
        self.mvp = Pool(self.node)

        ## Run
        if self.thisNode:
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.spinThread = threading.Thread(daemon = True, target = self.executor.spin)
            self.spinThread.start()
            self.node.get_logger().info('[~] rtsp running alone')


    def cback_Sub(self, msg: String):
        raw = msg.data.strip()

        ## yaml parsing
        try:
            yml = yaml.safe_load(raw)
            if isinstance(yml, dict):
                self.configChange(height = yml.get('height'),
                                  jpegQuality = yml.get('jpegQuality'),
                                  pubSpeed = yml.get('pubSpeed'),
                                  rtspUrl = yml.get('rtspUrl'),
                                  width = yml.get('width'))
                return
        except Exception as E:
            self.node.get_logger().warn(f'[!] yaml {E}')
        self.node.get_logger().error(f'[!] Input not understood {msg.data}')


    def paramHandler(self):
        if not self.node.has_parameter('codec'):
            self.node.declare_parameter('codec', 'h264')

        if not self.node.has_parameter('height'):
            self.node.declare_parameter('height', 480)
        self.height = int(self.args.get('height',
                                        self.node.get_parameter_or('height',
                                                                   480).value))

        if not self.node.has_parameter('jpegQuality'):
            self.node.declare_parameter('jpegQuality', 50)
        self.jpegQuality = int(self.args.get('jpegQuality',
                                             self.node.get_parameter_or('jpegQuality',
                                                                        50).value))

        if not self.node.has_parameter('pubSpeed'):
            self.node.declare_parameter('pubSpeed', 30)
        self.pubSpeed_hz = int(self.args.get('pubSpeed',
                                             self.node.get_parameter_or('pubSpeed',
                                                                        30).value))
        self.pubSpeed_period = 1.0 / self.pubSpeed_hz

        if not self.node.has_parameter('rtspUrl'):
            self.node.declare_parameter('rtspUrl', 'rtsp://127.0.0.1:8554/video')
        self.rtspUrl = self.args.get('rtspUrl',
                                     self.node.get_parameter_or('rtspUrl',
                                                                'rtsp://127.0.0.1:8554/video').value)

        if not self.node.has_parameter('topicId'):
            self.node.declare_parameter('topicId', '')
        self.topicId = self.args.get('topicId',
                                     self.node.get_parameter_or('topicId',
                                                                '').value)

        if not self.node.has_parameter('width'):
            self.node.declare_parameter('width', 640)
        self.width = int(self.args.get('width',
                                       self.node.get_parameter_or('width',
                                                                  640).value))


    def busErrors(self, bus, message):
        msgType = message.type
        if msgType == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.node.get_logger().error(f'[!] GST error {err.message} from {message.src.get_name()}')
            if debug:
                self.node.get_logger().error(f'[~] debug {debug}')
            self.pipelineRestart()
        elif msgType == Gst.MessageType.EOS:
            self.node.get_logger().warn('[~] GST EOS')
            self.pipelineRestart()


    def newSample(self, sink):
        if self.flag_Shutdown.is_set():
            return Gst.FlowReturn.FLUSHING

        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        try:
            h = caps.get_structure(0).get_value('height')
            w = caps.get_structure(0).get_value('width')

            if self.height != int(h) or self.width != int(w):
                self.height, self.width = int(h), int(w)
                self.node.get_logger().info(f'[~] Resolution now {self.width}x{self.height}')

            arr = np.frombuffer(mapinfo.data, dtype = np.uint8)

            ### Future planning for other caps
            fmt = caps.get_structure(0).get_value('format')
            if fmt not in ('RGB', 'BGR'):
                return Gst.FlowReturn.ERROR
            frame = arr.reshape((int(h), int(w), 3))

            ## latest
            now = time.time()
            with self.lock_Snapshot:
                self.new_Snapshot = frame.copy()
                self.new_Timestamp = now

        except Exception as E:
            self.node.get_logger().warn(f'[!] {E}')
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK


    def topicHandler(self):
        self.pub_Compressed = self.node.create_publisher(CompressedImage,
                                                         f'/captures/rtsp/compressed{self.topicId}',
                                                         self.qos)
        self.pub_Pickled = self.node.create_publisher(UInt8MultiArray,
                                                      f'/captures/rtsp/pickled{self.topicId}',
                                                      self.qos)
        self.pub_Raw = self.node.create_publisher(Image,
                                                  f'/captures/rtsp/raw{self.topicId}',
                                                  self.qos)
        self.sub_Config = self.node.create_subscription(String,
                                                        f'/captures/rtsp/config{self.topicId}',
                                                        self.cback_Sub,
                                                        self.qos)


    def rtspThread(self):
        """Threads out GStreamer frames for RTSP use"""
        while not self.flag_Shutdown.is_set():
            try:
                chain = self.chainDecoder.build()
                rLoc = f'rtspsrc location={self.rtspUrl} {chain}'
                with self.lock_Thread:
                    self.pipeline_Gstreamer = Gst.parse_launch(rLoc)
                    self.sink_App = self.pipeline_Gstreamer.get_by_name('appsink')
                    self.sink_App.connect('new-sample', self.newSample)

                ## Errors
                bus = self.pipeline_Gstreamer.get_bus()
                bus.add_signal_watch()
                bus.connect('message', self.busErrors)

                ## Run
                self.pipeline_Gstreamer.set_state(Gst.State.PLAYING)
                self.loop_GLib = GLib.MainLoop()
                self.loop_GLib.run()
                self.node.get_logger().info(f'[~] Pipeline {self.rtspUrl}')

            except Exception as E:
                self.node.get_logger().error(f'[!] Pipeline {E}')

            ## Cleanup
            try:
                if self.pipeline_Gstreamer:
                    self.pipeline_Gstreamer.set_state(Gst.State.NULL)
                self.pipeline_Gstreamer = None
                self.sink_App = None
            except Exception as E:
                self.node.get_logger().error(f'[!] Cleanup {E}')

            if not self.flag_Shutdown.is_set():
                self.node.get_logger().warn(f'[~] Attempting reconnect')
                time.sleep(1)


    def pipelineRestart(self):
        with self.lock_Thread:
            try:
                if self.loop_GLib:
                    GLib.idle_add(self.loop_GLib.quit)
                if self.pipeline_Gstreamer:
                    self.pipeline_Gstreamer.set_state(Gst.State.NULL)
            except Exception as E:
                self.node.get_logger().error(f'[~] Restart fail {E}')


    def configChange(self, height = None, jpegQuality = None, pubSpeed = None, rtspUrl = None, width = None):
        """Reconfigure RTSP capture settings"""
        needsRestart = False

        if height is not None:
            height = int(height)
            if height > 0 and height != self.height:
                self.node.get_logger().info(f'[~] height now {height}')
                self.height = height
                needsRestart = True


        if jpegQuality is not None:
            try:
                jpegQuality = int(jpegQuality)
                if jpegQuality > 0 and jpegQuality != self.jpegQuality:
                    self.jpegQuality = jpegQuality
                    self.node.set_parameters([rclpy.parameter.Parameter('jpegQuality', value = jpegQuality)])
                    self.cEncode = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpegQuality]
                    self.node.get_logger().info(f'[~] jpegQuality now {jpegQuality}')
            except Exception as E:
                self.node.get_logger().error(f'[!] jpegQuality {E}')

        if pubSpeed is not None:
            try:
                pubSpeed_hz = int(pubSpeed)
                if pubSpeed_hz <= 0:
                    raise ValueError
            except Exception as E:
                pubSpeed_hz = None
                self.node.get_logger().error(f'[!] pubSpeed {E}')
                
            if pubSpeed_hz and pubSpeed_hz != self.pubSpeed_hz:
                self.pubSpeed_hz = pubSpeed_hz
                self.pubSpeed_period = 1.0 / pubSpeed_hz

                # update ROS parameter
                self.node.set_parameters([rclpy.parameter.Parameter('pubSpeed', value = pubSpeed_hz)])

                # update timer period
                try:
                    self.timer.cancel()
                except Exception as E:
                    self.node.get_logger().error(f'[!] Timer {E}')
                try:
                    self.node.destroy_timer(self.timer)
                except Exception as E:
                    self.node.get_logger().error(f'[!] Timer {E}')
                self.timer = self.node.create_timer(self.pubSpeed_period, self.cback_Pub)
                self.node.get_logger().info(f'[~] pubSpeed now {pubSpeed_hz}')

        if width is not None:
            width = int(width)
            if width > 0 and width != self.width:
                self.node.get_logger().info(f'[~] width now {width}')
                self.width = width
                needsRestart = True

        if rtspUrl is not None:
            if rtspUrl != self.rtspUrl:
                self.node.get_logger().info(f'[~] rtspUrl now {rtspUrl}')
                self.rtspUrl = rtspUrl
                needsRestart = True

        if needsRestart:
            self.node.get_logger().warn('[~] Restarting GStreamer pipeline due to config change...')
            self.chainDecoder = self.chainDecoder.update(height = self.height, width = self.width, )
            self.pipelineRestart()


    def cback_Pub(self):
        """Controls publish rate"""

        ## latest frame
        with self.lock_Snapshot:
            frame = self.new_Snapshot
        now = self.node.get_clock().now().to_msg()
        if frame is None:
            return

        ## compressed image
        if self.pub_Compressed.get_subscription_count() > 0:
            try:
                self.cMsg.header.stamp = now
                self.cMsg.data = cv2.imencode('.jpg', frame, self.cEncode)[1].tobytes()
                self.pub_Compressed.publish(self.cMsg)
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')

        ## pickled image
        if self.pub_Pickled.get_subscription_count() > 0:
            try:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    return
                obj = {'alt': self.mvp._alt.get('rel'),
                       'pubSpeed': self.pubSpeed_hz,
                       'frame': jpeg.tobytes(),
                       'height': self.height,
                       'lat': self.mvp._gps.get('lat'),
                       'lon': self.mvp._gps.get('lon'),
                       'tstamp': now,
                       'width': self.width}
                self.pMsg.data = list(pickle.dumps(obj))
                self.pub_Pickled.publish(self.pMsg)
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')

        ## raw image
        if self.pub_Raw.get_subscription_count() > 0:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding = 'bgr8')
                msg.header.stamp = now
                self.pub_Raw.publish(msg)
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')


    def shutdown(self):
        try:
            self.flag_Shutdown.set()
            if self.loop_GLib is not None:
                try:
                    self.loop_GLib.quit()
                except Exception as E:
                    self.node.get_logger().warn(f'[!] {E}')
            if self.pipeline_Gstreamer is not None:
                try:
                    self.pipeline_Gstreamer.set_state(Gst.State.NULL)
                except Exception as E:
                    self.node.get_logger().warn(f'[!] {E}')
            if self.thread_Gstreamer is not None and self.thread_Gstreamer.is_alive():
                self.thread_Gstreamer.join(timeout = 2.0)
        except Exception as E:
            self.node.get_logger().warn(f'[!] {E}')

        if self.thisNode:

            ## Handle multi
            try:
                self.executor.shutdown()
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')

            try:
                self.node.destroy_node()
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')
            rclpy.shutdown()


def main(args = None):
    rclpy.init(args = args)
    rtsp = Rtsp()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rtsp.shutdown()

if __name__ == '__main__':
    main()
