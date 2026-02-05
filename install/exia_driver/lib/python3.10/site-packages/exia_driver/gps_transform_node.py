#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Header

import tf2_ros


class GpsTransformNode(Node):

    def __init__(self):
        super().__init__('gps_transform_node')

        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('auto_set_origin', True)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('gps_frame_id', 'gps_link')
        self.declare_parameter('odom_frame_id', 'odom')

        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.auto_set_origin = self.get_parameter('auto_set_origin').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.gps_frame_id = self.get_parameter('gps_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        self.origin_set = False
        if self.origin_lat != 0.0 or self.origin_lon != 0.0:
            self.origin_set = True
            self.get_logger().info(
                f'Using configured origin: lat={self.origin_lat}, lon={self.origin_lon}')

        self.last_fix: Optional[NavSatFix] = None

        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self._gps_callback, gps_qos)

        self.odom_pub = self.create_publisher(Odometry, '/gps/odom', 10)

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('GPS Transform Node initialized')

    def _gps_callback(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return

        if not self.origin_set:
            if self.auto_set_origin:
                self.origin_lat = msg.latitude
                self.origin_lon = msg.longitude
                self.origin_set = True
                self.get_logger().info(
                    f'GPS origin auto-set: lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}')
            else:
                return

        x, y = self._gps_to_local(msg.latitude, msg.longitude)

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.gps_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = msg.altitude if msg.altitude != 0.0 else 0.0
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        position_cov = max(msg.position_covariance[0], 0.1)
        odom.pose.covariance = [
            position_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, position_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, position_cov * 4, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e6,
        ]

        self.odom_pub.publish(odom)

        if self.publish_tf:
            self._publish_tf(odom)

        self.last_fix = msg

    def _gps_to_local(self, lat: float, lon: float) -> tuple:
        lat0_rad = math.radians(self.origin_lat)

        x = (lon - self.origin_lon) * math.cos(lat0_rad) * 111320.0
        y = (lat - self.origin_lat) * 111320.0

        return x, y

    def _publish_tf(self, odom: Odometry):
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GpsTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
