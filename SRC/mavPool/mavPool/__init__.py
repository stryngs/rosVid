#!/usr/bin/env python3

import math
import rclpy
import threading
import time
from geometry_msgs.msg import TwistStamped, Vector3
from mavros_msgs.msg import State, VfrHud
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, TimeReference
from std_msgs.msg import Float64, UInt32

class Pool():
    def __init__(self, node: Node | None = None, vehicleMass = 1000.0):
        """
        If the node is provided it will attach and share with that node.
        If the node is not provided this class will create its own node.
        """
        self.node = node
        self.thisNode = node is None

        ## kilos and gps
        self.vehicleMass = vehicleMass / 1000.0
        self.fDict = {-1: 'No Fix',
                      0: '3D Fix',
                      1: 'DGPS',
                      2: 'RTK Float',
                      5: 'RTK Fixed'}

        ## Setup defaults for structure layout
        self._alt = {'gps': None, 'rel': None, 'vfr': None}
        self._batt = {'current': None, 'temperature': None, 'voltage': None}
        self._compass = {'hdg': None}
        self._gps = {'count': 0, 'fix': 'Unknown', 'lat': None, 'lon': None}
        self._imu = {'pitch': None, 'roll': None, 'yaw': None}
        self._state = {'armed': None, 'mode': None}
        self._time = {'epoch': None}
        self._vel = {'ke': None, 'm/s': None}
        self.lastGps = None
        self.lastGps_time = None

        ## Decide if this node
        if self.thisNode:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node('mavPool')
        self.topicHandler()

        ## Run
        if self.thisNode:
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self._spin_thread = threading.Thread(daemon = True, target = self.executor.spin)
            self._spin_thread.start()
            self.node.get_logger().info('[~] mavPool running alone')


    def cback_Alt(self, msg):
        self._alt['rel'] = msg.data


    def cback_Batt(self, msg):
        self._batt.update({'current': msg.current,
                           'temperature': msg.temperature,
                           'voltage': msg.voltage})


    def cback_Compass(self, msg):
        self._compass['hdg'] = msg.data


    def cback_Gps(self, msg):
        self._gps.update({'lat': msg.latitude, 'lon': msg.longitude})
        self._alt['gps'] = msg.altitude
        now = time.time()
        if self.lastGps and self.lastGps_time:
            dt = now - self.lastGps_time
            if dt > 0:
                lat1, lon1 = math.radians(self.lastGps.latitude), math.radians(self.lastGps.longitude)
                lat2, lon2 = math.radians(msg.latitude), math.radians(msg.longitude)
                dlat, dlon = lat2 - lat1, lon2 - lon1
                a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
                R = 6371000.0
                dst = R * c
                speed = dst / dt
        self.lastGps, self.lastGps_time = msg, now


    def cback_GpsCount(self, msg):
        self._gps['count'] = msg.data


    def cback_GpsFix(self, msg):
        self._gps['fix'] = self.fDict.get(msg.status.status, 'N/A')


    def cback_Imu(self, msg):
        roll, pitch, yaw = self.quaternionEuler(msg.orientation.x,
                                                msg.orientation.y,
                                                msg.orientation.z,
                                                msg.orientation.w)
        self._imu.update({'pitch': pitch, 'roll': roll, 'yaw': yaw})


    def cback_State(self, msg):
        self._state.update({'armed': msg.armed, 'mode': msg.mode})


    def cback_Time(self, msg):
        unix_usec = msg.time_ref.sec * 1000000 + msg.time_ref.nanosec // 1000
        self._time['epoch'] = unix_usec


    def cback_Vel(self, msg):
        x, y, z = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
        mag = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        ke = 0.5 * self.vehicleMass * mag ** 2
        self._vel.update({'ke': ke, 'm/s': mag})


    def cback_Vfr(self, msg):
        self._alt['vfr'] = msg.altitude


    def quaternionEuler(self, qx, qy, qz, qw):
        """Angles of bank"""
        ## pitch
        sinP = 2 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi/2, sinP) if abs(sinP) >= 1 else math.asin(sinP)

        ## roll
        sinR_cosp = 2 * (qw * qx + qy * qz)
        cosR_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinR_cosp, cosR_cosp)

        ## yaw
        sinY_cosp = 2 * (qw * qz + qx * qy)
        cosY_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(sinY_cosp, cosY_cosp)
        return roll, pitch, yaw


    def topicHandler(self):
        qos = QoSProfile(depth = 10, history = HistoryPolicy.KEEP_LAST, reliability = ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Float64, '/mavros/global_position/rel_alt', self.cback_Alt, qos)
        self.node.create_subscription(BatteryState, '/mavros/battery', self.cback_Batt, qos)
        self.node.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.cback_Compass, qos)
        self.node.create_subscription(NavSatFix, '/mavros/global_position/global', self.cback_Gps, qos)
        self.node.create_subscription(UInt32, '/mavros/global_position/raw/satellites', self.cback_GpsCount, qos)
        self.node.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self.cback_GpsFix, qos)
        self.node.create_subscription(Imu, '/mavros/imu/data', self.cback_Imu, qos)
        self.node.create_subscription(State, '/mavros/state', self.cback_State, qos)
        self.node.create_subscription(TimeReference, '/mavros/time_reference', self.cback_Time, qos)
        self.node.create_subscription(TwistStamped, '/mavros/local_position/velocity_body', self.cback_Vel, qos)
        self.node.create_subscription(VfrHud, '/mavros/vfr_hud', self.cback_Vfr, qos)


    def shutdown(self):
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

if __name__ == '__main__':
    mavp = Mavpool()
    while True:
        time.sleep(1)
