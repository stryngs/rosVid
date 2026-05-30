import pickle
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8MultiArray

print(f'[TRACE ros] imported {__file__}', flush = True)


class TopicFrameBridge(Node):
    """Subscribe to a ROS topic and expose the latest decoded video frame."""
    def __init__(self, topic, width, height, topicType = 'UInt8MultiArray', frameKey = 'frame', node_name = 'rtsp_topic_bridge'):
        super().__init__(node_name)
        self.topic = topic
        self.topicType = self.normalizeTopicType(topicType)
        self.frameKey = str(frameKey)
        self.width = int(width)
        self.height = int(height)
        self.frame = None
        self.frameLock = threading.Lock()
        self.cv2 = None
        self.np = None
        self.didTraceDeps = False
        self.didTraceFrame = False
        self.didTraceFallback = False
        self.bridge = CvBridge() if self.topicType == 'Image' else None

        qos = QoSProfile(depth = 10,
                         history = HistoryPolicy.KEEP_LAST,
                         reliability = ReliabilityPolicy.BEST_EFFORT)
        print((f'[TRACE ros] TopicFrameBridge init topic={topic!r} '
               f'type={self.topicType!r} '
               f'key={self.frameKey!r} '
               f'width={self.width} height={self.height} '
               f'node={node_name!r}'), flush = True)
        msgType, callback = self.topicSubscription()
        self.subscription = self.create_subscription(msgType,
                                                     topic,
                                                     callback,
                                                     qos)
        self.get_logger().info(f'[~] subscribed {topic}')


    def initFrameDeps(self):
        """Lazy-load frame dependencies after ROS/GStreamer have initialized."""
        if self.cv2 is None or self.np is None:
            import cv2
            import numpy as np
            self.cv2 = cv2
            self.np = np
            if not self.didTraceDeps:
                print('[TRACE ros] loaded cv2/numpy frame dependencies', flush = True)
                self.didTraceDeps = True


    def normalizeTopicType(self, topicType):
        """Normalize CLI-friendly type names to ROS message families."""
        normalized = str(topicType).strip()
        aliases = {'UInt8MultiArray': 'UInt8MultiArray',
                   'CompressedImage': 'CompressedImage',
                   'Image': 'Image'}
        return aliases.get(normalized, normalized)


    def topicSubscription(self):
        """Return the ROS message type and decoder for the configured topic type."""
        if self.topicType == 'UInt8MultiArray':
            return UInt8MultiArray, self.cbackPickled
        if self.topicType == 'CompressedImage':
            return CompressedImage, self.cbackCompressed
        if self.topicType == 'Image':
            return Image, self.cbackRaw
        raise ValueError(f'[!] unsupported topic type {self.topicType!r}')


    def cbackPickled(self, msg):
        """Decode a pickled JPEG frame from a UInt8MultiArray topic."""
        try:
            self.initFrameDeps()
            payload = pickle.loads(bytes(msg.data))
            jpeg = payload.get(self.frameKey, b'')
            frame = self.decodeJpeg(jpeg)
            if frame is None:
                print((f'[TRACE ros] decoded empty pickled frame '
                       f'key={self.frameKey!r} payload_keys={list(payload.keys())} '
                       f'jpeg_bytes={len(jpeg)}'), flush = True)
                return
            self.storeFrame(frame, trace = (f'key={self.frameKey!r} '
                                           f'payload_keys={list(payload.keys())} '
                                           f'jpeg_bytes={len(jpeg)}'))
        except Exception as exc:
            print(f'[TRACE ros] pickled decode exception {exc}', flush = True)
            self.get_logger().warn(f'[!] pickled topic decode {exc}')


    def cbackCompressed(self, msg):
        """Decode a sensor_msgs/CompressedImage topic."""
        try:
            self.initFrameDeps()
            frame = self.decodeJpeg(msg.data)
            if frame is None:
                print(f'[TRACE ros] decoded empty compressed frame jpeg_bytes={len(msg.data)}', flush = True)
                return
            self.storeFrame(frame, trace = f'jpeg_bytes={len(msg.data)}')
        except Exception as exc:
            print(f'[TRACE ros] compressed decode exception {exc}', flush = True)
            self.get_logger().warn(f'[!] compressed topic decode {exc}')


    def cbackRaw(self, msg):
        """Decode a sensor_msgs/Image topic."""
        try:
            self.initFrameDeps()
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            self.storeFrame(frame, trace = f'encoding={getattr(msg, "encoding", "")}')
        except Exception as exc:
            print(f'[TRACE ros] raw decode exception {exc}', flush = True)
            self.get_logger().warn(f'[!] raw topic decode {exc}')


    def decodeJpeg(self, jpeg):
        """Decode JPEG bytes into a BGR frame."""
        arr = self.np.frombuffer(jpeg, self.np.uint8)
        return self.cv2.imdecode(arr, self.cv2.IMREAD_COLOR)


    def storeFrame(self, frame, trace = ''):
        """Resize and store the latest decoded BGR frame."""
        if frame.shape[0] != self.height or frame.shape[1] != self.width:
            print((f'[TRACE ros] resizing frame from '
                   f'{frame.shape[1]}x{frame.shape[0]} to '
                   f'{self.width}x{self.height}'), flush = True)
            frame = self.cv2.resize(frame, (self.width, self.height))

        with self.frameLock:
            self.frame = self.np.ascontiguousarray(frame)
        if not self.didTraceFrame:
            print((f'[TRACE ros] first topic frame received type={self.topicType!r} '
                   f'{trace} frame_shape={frame.shape}'), flush = True)
            self.didTraceFrame = True


    def latestFrame(self):
        """Return the latest decoded frame, or a black frame before input arrives."""
        self.initFrameDeps()
        with self.frameLock:
            if self.frame is not None:
                return self.frame.copy()
        if not self.didTraceFallback:
            print('[TRACE ros] latestFrame using black fallback; no topic frame received yet', flush = True)
            self.didTraceFallback = True
        return self.np.zeros((self.height, self.width, 3), dtype = self.np.uint8)
