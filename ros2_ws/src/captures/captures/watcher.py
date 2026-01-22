#!/usr/bin/env python3

import cv2
import numpy as np
import pickle
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge

class Viewer(Node):
    def __init__(self):
        super().__init__('watcher')
        self.capture = self.declare_parameter('capture', 'phys').value
        self.compressed = self.declare_parameter('compressed', False).value
        self.pickled = self.declare_parameter('pickled', False).value
        self.topicId = self.declare_parameter('topicId', '').value
        self.windowName = self.declare_parameter('windowName', '').value
        qos = QoSProfile(depth = 10,
                         history = HistoryPolicy.KEEP_LAST,
                         reliability = ReliabilityPolicy.BEST_EFFORT)

        ## compressed
        if self.compressed:
            self.bridge = CvBridge()
            self.sub = self.create_subscription(CompressedImage,
                                                f'/captures/{self.capture}/compressed{self.topicId}',
                                                self.cback_Compressed,
                                                qos)

        ## pickled
        elif self.pickled and not self.compressed:
            self.sub = self.create_subscription(UInt8MultiArray,
                                                f'/captures/{self.capture}/pickled{self.topicId}',
                                                self.cback_Pickled,
                                                qos)

        ## raw
        else:
            self.bridge = CvBridge()
            self.sub = self.create_subscription(Image,
                                                f'/captures/{self.capture}/raw{self.topicId}',
                                                self.cback_Raw,
                                                qos)


    def cback_Pickled(self, msg):
        try:
            pck = pickle.loads(bytes(msg.data))
            pubSpeed = pck.get('pubSpeed')
            if pubSpeed is None:
                pubSpeed = 0
            jpeg_bytes = pck.get('frame', b'')
            np_arr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn('[!] Decode fail')
                return

            text = f'pubSpeed @ {pubSpeed}'
            cv2.putText(frame,
                        text,
                        (10, 20), # px location
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, # scale
                        (0, 255, 0), # rgb
                        1) # thickness

            cv2.imshow(self.windowName, frame)
            cv2.waitKey(1)
        except Exception as E:
            self.get_logger().error(f'[!] {E}')


    def cback_Raw(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            cv2.imshow(self.windowName, frame)
            cv2.waitKey(1)
        except Exception as E:
            self.get_logger().error(f'[!] {E}')


    def cback_Compressed(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow(self.windowName, frame)
            cv2.waitKey(1)
        except Exception as E:
            self.get_logger().error(f'[!] {E}')


def main(args = None):
    rclpy.init(args = args)
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
