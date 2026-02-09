#!/usr/bin/env python3

import cv2
import pickle
import rclpy
import threading
import time
import yaml
from cv_bridge import CvBridge
from libStreamer import Config
from libStreamer.lib.opencv import CaptureHandler
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String, UInt8MultiArray

class Phys:
    """Handler for capturing a stream from a physically attached camera"""
    def __init__(self, node: Node | None = None, args = None):
        """
        If the node is provided it will attach and share with that node.
        If the node is not provided this class will create its own Node.
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
            self.node = Node('phys')
        
        ## Param/Topic handlers
        self.paramHandler()
        self.topicHandler()

        ## Interface setup
        self.cMsg = CompressedImage()
        self.cMsg.format = 'jpeg'
        self.pMsg = UInt8MultiArray()

        ## libStreamer integration
        self.cEncode = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpegQuality]
        self.capLock = threading.Lock()
        self.bridge = CvBridge()
        self.camConfig = Config(device = self.device,
                                fps = self.fps,
                                height = self.height,
                                width = self.width)
        self.camCapture = CaptureHandler(self.camConfig)

        ## Open the capture
        with self.capLock:
            self.cap = self.camCapture.openCap()
            if self.cap is None:
                self.node.get_logger().error(f'[!] Capture issue with {self.device}')

        ## Callbacks
        self.timer = self.node.create_timer(self.pubSpeed_period, self.cback_Pub)

        ## mavPool connection
        if self.mavPool:
            from mavPool import Pool
            self.mvp = Pool(self.node)

        ## Run
        if self.thisNode:
            self.theThread = threading.Thread(daemon = True, target = lambda: rclpy.spin(self.node))
            self.theThread.start()
            self.node.get_logger().info('[~] phys running alone')


    def cback_Sub(self, msg: String):
        raw = msg.data.strip()

        ## yaml parsing
        try:
            yml = yaml.safe_load(raw)
            if isinstance(yml, dict):
                self.configChange(device = yml.get('device'),
                                  fps = yml.get('fps'),
                                  height = yml.get('height'),
                                  jpegQuality = yml.get('jpegQuality'),
                                  pubSpeed = yml.get('pubSpeed'),
                                  width = yml.get('width'))
                return
        except Exception as E:
            self.node.get_logger().warn(f'[!] yaml {E}')
        self.node.get_logger().error(f'[!] Input not understood {msg.data}')


    def paramHandler(self):
        if not self.node.has_parameter('device'):
            self.node.declare_parameter('device', '/dev/video0')
        self.device = self.args.get('device',
                                    self.node.get_parameter_or('device',
                                                               '/dev/video0').value)

        if not self.node.has_parameter('fps'):
            self.node.declare_parameter('fps', 30)
        self.fps = int(self.args.get('fps',
                                     self.node.get_parameter_or('fps', 30).value))

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

        if not self.node.has_parameter('mavPool'):
            self.node.declare_parameter('mavPool', False)
        self.mavPool = self.args.get('mavPool',
                                     self.node.get_parameter_or('mavPool',
                                                                False).value)

        if not self.node.has_parameter('pubSpeed'):
            self.node.declare_parameter('pubSpeed', 30)
        self.pubSpeed_hz = int(self.args.get('pubSpeed',
                                             self.node.get_parameter_or('pubSpeed',
                                                                        30).value))
        self.pubSpeed_period = 1.0 / self.pubSpeed_hz

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


    def topicHandler(self):
        self.pub_Compressed = self.node.create_publisher(CompressedImage,
                                                         f'captures/phys/compressed{self.topicId}',
                                                         self.qos)
        self.pub_Pickled = self.node.create_publisher(UInt8MultiArray,
                                                      f'/captures/phys/pickled{self.topicId}',
                                                      self.qos)
        self.pub_Raw = self.node.create_publisher(Image,
                                                  f'/captures/phys/raw{self.topicId}',
                                                  self.qos)
        self.sub_Config = self.node.create_subscription(String,
                                                        f'/captures/phys/config{self.topicId}',
                                                        self.cback_Sub,
                                                        self.qos)


    def reCapture(self, cfg):
        """Reopen the capture"""
        with self.capLock:
            try:
                if self.cap is not None:
                    self.cap.release()
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')
            cap = cv2.VideoCapture(cfg.device)
            if not cap.isOpened():
                self.node.get_logger().error(f'[!] Cannot open {cfg.device}')
                return None
            self.camCapture.applySettings(cap)
            self.cap = cap
            return self.cap


    def configChange(self, device = None, fps = None, height = None, jpegQuality = None, pubSpeed = None, width = None):
        """Reconfigure camera settings"""
        updates = {}
        
        if device is not None and device != self.device:
            updates['device'] = device
            self.node.set_parameters([rclpy.parameter.Parameter('device', value = device)])
            self.node.get_logger().info(f'[~] device now {device}')

        if fps is not None:
            fps = int(fps)
            if fps > 0 and fps != self.fps:
                updates['fps'] = fps
                self.node.set_parameters([rclpy.parameter.Parameter('fps', value = fps)])
                self.node.get_logger().info(f'[~] fps now {fps}')

        if height is not None:
            height = int(height)
            if height != self.height:
                updates['height'] = height
                self.node.set_parameters([rclpy.parameter.Parameter('height', value = height)])
                self.node.get_logger().info(f'[~] height now {height}')

        if width is not None:
            width = int(width)
            if width != self.width:
                updates['width'] = width
                self.node.set_parameters([rclpy.parameter.Parameter('width', value = width)])
                self.node.get_logger().info(f'[~] width now {width}')

        if updates:
            updCam, needsRestart = self.camCapture.updateChecks(updates, reCapture = self.reCapture)
            self.camCapture = updCam
            self.device = self.camCapture.config.device
            self.fps = self.camCapture.config.fps
            self.height = self.camCapture.config.height
            self.width = self.camCapture.config.width

            if not needsRestart and 'fps' in updates:
                with self.capLock:
                    if self.cap is not None:
                        self.camCapture.applySettings(self.cap)

        if jpegQuality is not None:
            jpegQuality = int(jpegQuality)
            if jpegQuality > 0 and jpegQuality != self.jpegQuality:
                self.jpegQuality = jpegQuality
                self.node.set_parameters([rclpy.parameter.Parameter('jpegQuality', value = jpegQuality)])
                self.cEncode = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpegQuality]           
                self.node.get_logger().info(f'[~] jpegQuality now {jpegQuality}')

        if pubSpeed is not None:
            pubSpeed = int(pubSpeed)
            if pubSpeed > 0:
                self.pubSpeed_hz = pubSpeed
                self.pubSpeed_period = 1.0 / pubSpeed

                self.node.set_parameters([rclpy.parameter.Parameter('pubSpeed', value = pubSpeed)])

                ## reset timer
                try:
                    self.timer.cancel()
                except Exception as E:
                    self.node.get_logger().error(f'[!] Cancel fail {E}')
                try:
                    self.node.destroy_timer(self.timer)
                except Exception as E:
                    self.node.get_logger().error(f'[!] Timer fail {E}')
                self.timer = self.node.create_timer(self.pubSpeed_period, self.cback_Pub)
                self.node.get_logger().info(f'[~] pubSpeed {pubSpeed}')


    def cback_Pub(self):
        """Controls publish rate"""

        ## latest frame
        with self.capLock:
            ret, frame = self.cap.read()
        now = self.node.get_clock().now().to_msg()
        if not ret:
            return

        ## compressed
        if self.pub_Compressed.get_subscription_count() > 0:
            try:
                self.cMsg.header.stamp = now
                self.cMsg.data = cv2.imencode('.jpg', frame, self.cEncode)[1].tobytes()
                self.pub_Compressed.publish(self.cMsg)
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')
        
        ## pickled
        if self.pub_Pickled.get_subscription_count() > 0:
            try:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    return
                if self.mavPool:
                    alt = self.mvp._alt.get('rel')
                    lat = self.mvp._gps.get('lat')
                    lon = self.mvp._gps.get('lon')
                else:
                    alt = 'N/A'
                    lat = 'N/A'
                    lon = 'N/A'
                obj = {'alt': alt,
                       'frame': jpeg.tobytes(),
                       'fps': self.fps,
                       'height': self.height,
                       'lat': lat,
                       'lon': lon,
                       'pubSpeed': self.pubSpeed_hz,
                       'tstamp': now,
                       'width': self.width}
                self.pMsg.data = list(pickle.dumps(obj))
                self.pub_Pickled.publish(self.pMsg)
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')
        
        ## raw
        if self.pub_Raw.get_subscription_count() > 0:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding = 'bgr8')
                msg.header.stamp = now
                self.pub_Raw.publish(msg)
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')


    def shutdown(self):
        if self.thisNode:
            try:
                self.node.destroy_node()
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')
            rclpy.shutdown()


def main(args = None):
    rclpy.init(args = args)
    phys = Phys()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        phys.shutdown()

if __name__ == '__main__':
    main()
