#!/usr/bin/env python3

import cv2
import pickle
import rclpy
import threading
import time
import yaml
import numpy as np
from array import array
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String, UInt8MultiArray
from urllib.request import Request, urlopen

class Http:
    """Handler for capturing JPEG or MJPEG frames from an HTTP endpoint."""
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
            self.node = Node('http')

        ## Param/Topic handlers
        self.paramHandler()
        self.topicHandler()

        ## Interface setup
        self.cMsg = CompressedImage()
        self.cMsg.format = 'jpeg'
        self.pMsg = UInt8MultiArray()
        self.cEncode = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpegQuality]
        self.bridge = CvBridge()

        ## HTTP integration
        self.flag_Shutdown = threading.Event()
        self.flag_Restart = threading.Event()
        self.lock_Snapshot = threading.Lock()
        self.new_Snapshot = None
        self.lastFrameTime = 0.0
        self.thread_Http = threading.Thread(daemon = True, target = self.httpThread)
        self.thread_Http.start()

        ## Callbacks
        self.timer = self.node.create_timer(self.pubSpeed_period, self.cback_Pub)

        ## mavPool connection
        if self.mavPool:
            from mavPool import Pool
            self.mvp = Pool(self.node)

        ## Run
        if self.thisNode:
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.spinThread = threading.Thread(daemon = True, target = self.executor.spin)
            self.spinThread.start()
            self.node.get_logger().info('[~] http running alone')


    def cback_Sub(self, msg: String):
        raw = msg.data.strip()

        ## yaml parsing
        try:
            yml = yaml.safe_load(raw)
            if isinstance(yml, dict):
                self.configChange(fps = yml.get('fps'),
                                  height = yml.get('height'),
                                  httpMode = yml.get('httpMode'),
                                  httpUrl = yml.get('httpUrl'),
                                  jpegQuality = yml.get('jpegQuality'),
                                  pubSpeed = yml.get('pubSpeed'),
                                  timeout = yml.get('timeout'),
                                  width = yml.get('width'))
                return
        except Exception as E:
            self.node.get_logger().warn(f'[!] yaml {E}')
        self.node.get_logger().error(f'[!] Input not understood {msg.data}')


    def paramHandler(self):
        if not self.node.has_parameter('fps'):
            self.node.declare_parameter('fps', 10)
        self.fps = int(self.args.get('fps',
                                     self.node.get_parameter_or('fps',
                                                                10).value))

        if not self.node.has_parameter('height'):
            self.node.declare_parameter('height', 480)
        self.height = int(self.args.get('height',
                                        self.node.get_parameter_or('height',
                                                                   480).value))

        if not self.node.has_parameter('httpMode'):
            self.node.declare_parameter('httpMode', 'auto')
        self.httpMode = str(self.args.get('httpMode',
                                          self.node.get_parameter_or('httpMode',
                                                                     'auto').value)).lower()

        if not self.node.has_parameter('httpUrl'):
            self.node.declare_parameter('httpUrl', 'http://127.0.0.1:8002/frame.jpg')
        self.httpUrl = self.args.get('httpUrl',
                                     self.node.get_parameter_or('httpUrl',
                                                                'http://127.0.0.1:8002/frame.jpg').value)

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

        if not self.node.has_parameter('timeout'):
            self.node.declare_parameter('timeout', 5.0)
        self.timeout = float(self.args.get('timeout',
                                           self.node.get_parameter_or('timeout',
                                                                      5.0).value))

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
                                                         f'/captures/http/compressed{self.topicId}',
                                                         self.qos)
        self.pub_Pickled = self.node.create_publisher(UInt8MultiArray,
                                                      f'/captures/http/pickled{self.topicId}',
                                                      self.qos)
        self.pub_Raw = self.node.create_publisher(Image,
                                                  f'/captures/http/raw{self.topicId}',
                                                  self.qos)
        self.sub_Config = self.node.create_subscription(String,
                                                        f'/captures/http/config{self.topicId}',
                                                        self.cback_Sub,
                                                        self.qos)


    def request(self):
        return Request(self.httpUrl,
                       headers = {'User-Agent': 'rosVid-captures-http/0.1',
                                  'Cache-Control': 'no-cache'})


    def httpThread(self):
        """Thread HTTP frame ingestion with reconnect."""
        while not self.flag_Shutdown.is_set():
            self.flag_Restart.clear()
            try:
                mode = self.httpMode.lower()
                if mode == 'snapshot':
                    self.snapshotLoop()
                elif mode == 'mjpeg':
                    self.mjpegLoop()
                else:
                    self.autoLoop()
            except Exception as E:
                self.node.get_logger().warn(f'[!] HTTP {E}')

            if not self.flag_Shutdown.is_set():
                self.node.get_logger().warn('[~] Attempting HTTP reconnect')
                time.sleep(1)


    def autoLoop(self):
        with urlopen(self.request(), timeout = self.timeout) as response:
            contentType = response.headers.get('Content-Type', '').lower()
            if 'multipart' in contentType:
                self.readMjpeg(response, contentType)
                return
            data = response.read()
            self.updateJpeg(data)
            self.snapshotLoop()


    def snapshotLoop(self):
        period = 1.0 / max(self.fps, 1)
        while not self.flag_Shutdown.is_set() and not self.flag_Restart.is_set():
            start = time.time()
            with urlopen(self.request(), timeout = self.timeout) as response:
                self.updateJpeg(response.read())
            delay = period - (time.time() - start)
            if delay > 0:
                self.flag_Restart.wait(delay)


    def mjpegLoop(self):
        with urlopen(self.request(), timeout = self.timeout) as response:
            contentType = response.headers.get('Content-Type', '').lower()
            self.readMjpeg(response, contentType)


    def readMjpeg(self, response, contentType):
        boundary = self.boundaryFromContentType(contentType)
        if boundary:
            self.readMjpegBoundary(response, boundary)
            return
        self.readMjpegScan(response)


    def boundaryFromContentType(self, contentType):
        for part in contentType.split(';'):
            part = part.strip()
            if part.startswith('boundary='):
                boundary = part.split('=', 1)[1].strip().strip('"')
                if not boundary.startswith('--'):
                    boundary = f'--{boundary}'
                return boundary.encode('ascii', errors = 'ignore')
        return None


    def readMjpegBoundary(self, response, boundary):
        buf = b''
        while not self.flag_Shutdown.is_set() and not self.flag_Restart.is_set():
            chunk = response.read(4096)
            if not chunk:
                return
            buf += chunk
            while True:
                start = buf.find(boundary)
                if start < 0:
                    buf = buf[-len(boundary):]
                    break
                end = buf.find(boundary, start + len(boundary))
                if end < 0:
                    buf = buf[start:]
                    break
                part = buf[start + len(boundary):end]
                buf = buf[end:]
                headerEnd = part.find(b'\r\n\r\n')
                if headerEnd < 0:
                    headerEnd = part.find(b'\n\n')
                if headerEnd < 0:
                    continue
                jpeg = part[headerEnd:].strip()
                if jpeg:
                    self.updateJpeg(jpeg)


    def readMjpegScan(self, response):
        buf = b''
        while not self.flag_Shutdown.is_set() and not self.flag_Restart.is_set():
            chunk = response.read(4096)
            if not chunk:
                return
            buf += chunk
            while True:
                start = buf.find(b'\xff\xd8')
                end = buf.find(b'\xff\xd9', start + 2)
                if start < 0 or end < 0:
                    buf = buf[-2:]
                    break
                jpeg = buf[start:end + 2]
                buf = buf[end + 2:]
                self.updateJpeg(jpeg)


    def updateJpeg(self, jpeg):
        if not jpeg:
            return
        arr = np.frombuffer(jpeg, dtype = np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.node.get_logger().warn('[!] JPEG decode failed')
            return
        if self.width > 0 and self.height > 0:
            h, w = frame.shape[:2]
            if int(w) != self.width or int(h) != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
        with self.lock_Snapshot:
            self.new_Snapshot = frame
            self.lastFrameTime = time.time()


    def configChange(self, fps = None, height = None, httpMode = None, httpUrl = None,
                     jpegQuality = None, pubSpeed = None, timeout = None, width = None):
        """Reconfigure HTTP capture settings."""
        needsRestart = False

        if fps is not None:
            fps = int(fps)
            if fps > 0 and fps != self.fps:
                self.fps = fps
                self.node.set_parameters([rclpy.parameter.Parameter('fps', value = fps)])
                self.node.get_logger().info(f'[~] fps now {fps}')

        if height is not None:
            height = int(height)
            if height > 0 and height != self.height:
                self.height = height
                self.node.set_parameters([rclpy.parameter.Parameter('height', value = height)])
                self.node.get_logger().info(f'[~] height now {height}')

        if httpMode is not None:
            httpMode = str(httpMode).strip().lower()
            if httpMode and httpMode != self.httpMode:
                self.httpMode = httpMode
                self.node.set_parameters([rclpy.parameter.Parameter('httpMode', value = httpMode)])
                self.node.get_logger().info(f'[~] httpMode now {httpMode}')
                needsRestart = True

        if httpUrl is not None:
            httpUrl = str(httpUrl).strip()
            if httpUrl and httpUrl != self.httpUrl:
                self.httpUrl = httpUrl
                self.node.set_parameters([rclpy.parameter.Parameter('httpUrl', value = httpUrl)])
                self.node.get_logger().info(f'[~] httpUrl now {httpUrl}')
                needsRestart = True

        if jpegQuality is not None:
            jpegQuality = int(jpegQuality)
            if jpegQuality > 0 and jpegQuality != self.jpegQuality:
                self.jpegQuality = jpegQuality
                self.node.set_parameters([rclpy.parameter.Parameter('jpegQuality', value = jpegQuality)])
                self.cEncode = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpegQuality]
                self.node.get_logger().info(f'[~] jpegQuality now {jpegQuality}')

        if pubSpeed is not None:
            pubSpeed = int(pubSpeed)
            if pubSpeed > 0 and pubSpeed != self.pubSpeed_hz:
                self.pubSpeed_hz = pubSpeed
                self.pubSpeed_period = 1.0 / pubSpeed
                self.node.set_parameters([rclpy.parameter.Parameter('pubSpeed', value = pubSpeed)])
                try:
                    self.timer.cancel()
                except Exception as E:
                    self.node.get_logger().error(f'[!] Timer {E}')
                try:
                    self.node.destroy_timer(self.timer)
                except Exception as E:
                    self.node.get_logger().error(f'[!] Timer {E}')
                self.timer = self.node.create_timer(self.pubSpeed_period, self.cback_Pub)
                self.node.get_logger().info(f'[~] pubSpeed now {pubSpeed}')

        if timeout is not None:
            timeout = float(timeout)
            if timeout > 0 and timeout != self.timeout:
                self.timeout = timeout
                self.node.set_parameters([rclpy.parameter.Parameter('timeout', value = timeout)])
                self.node.get_logger().info(f'[~] timeout now {timeout}')
                needsRestart = True

        if width is not None:
            width = int(width)
            if width > 0 and width != self.width:
                self.width = width
                self.node.set_parameters([rclpy.parameter.Parameter('width', value = width)])
                self.node.get_logger().info(f'[~] width now {width}')

        if needsRestart:
            self.node.get_logger().warn('[~] Restarting HTTP capture due to config change...')
            self.flag_Restart.set()


    def cback_Pub(self):
        """Controls publish rate."""
        with self.lock_Snapshot:
            frame = None if self.new_Snapshot is None else self.new_Snapshot.copy()
        now = self.node.get_clock().now().to_msg()
        if frame is None:
            return

        ## compressed image
        jpegBytes = None
        if self.pub_Compressed.get_subscription_count() > 0 or self.pub_Pickled.get_subscription_count() > 0:
            try:
                ret, jpeg = cv2.imencode('.jpg', frame, self.cEncode)
                if not ret:
                    return
                jpegBytes = jpeg.tobytes()
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')
                return

        if self.pub_Compressed.get_subscription_count() > 0:
            try:
                self.cMsg.header.stamp = now
                self.cMsg.data = jpegBytes
                self.pub_Compressed.publish(self.cMsg)
            except Exception as E:
                self.node.get_logger().warn(f'[!] {E}')

        ## pickled image
        if self.pub_Pickled.get_subscription_count() > 0:
            try:
                if self.mavPool:
                    alt = self.mvp._alt.get('rel')
                    lat = self.mvp._gps.get('lat')
                    lon = self.mvp._gps.get('lon')
                else:
                    alt = 'N/A'
                    lat = 'N/A'
                    lon = 'N/A'
                obj = {'alt': alt,
                       'fps': self.fps,
                       'frame': jpegBytes,
                       'height': self.height,
                       'httpMode': self.httpMode,
                       'httpUrl': self.httpUrl,
                       'lat': lat,
                       'lon': lon,
                       'pubSpeed': self.pubSpeed_hz,
                       'tstamp': now,
                       'width': self.width}
                self.pMsg.data = array('B', pickle.dumps(obj, protocol = pickle.HIGHEST_PROTOCOL))
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
            self.flag_Restart.set()
            if self.thread_Http is not None and self.thread_Http.is_alive():
                self.thread_Http.join(timeout = 2.0)
        except Exception as E:
            self.node.get_logger().warn(f'[!] {E}')

        if self.thisNode:
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
    http = Http()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        http.shutdown()


if __name__ == '__main__':
    main()
