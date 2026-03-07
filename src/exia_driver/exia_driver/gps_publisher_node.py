#!/usr/bin/env python3
import os
import math
import subprocess
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSPublisherNode(Node):
    def __init__(self):
        super().__init__('gps_publisher_node')

        self._gnss_proc = None
        self._init_gnss_driver()

        self.create_subscription(NavSatFix, '/navsatfix', self._gps_cb, 10)

        self.get_logger().info('GPS publisher node started — waiting for fix...')

    def _init_gnss_driver(self):
        ws = os.path.expanduser('~/exia_ws')
        candidates = [
            os.path.join(ws, 'src', 'exia_bringup', 'config', 'septentrio_rover.yaml'),
            os.path.join(ws, 'install', 'exia_bringup', 'share',
                         'exia_bringup', 'config', 'septentrio_rover.yaml'),
        ]
        params_file = None
        for c in candidates:
            if os.path.isfile(c):
                params_file = c
                break
        if params_file is None:
            self.get_logger().error(f'GPS params file not found, tried: {candidates}')
            return
        try:
            self._gnss_proc = subprocess.Popen(
                ['ros2', 'run', 'septentrio_gnss_driver',
                 'septentrio_gnss_driver_node',
                 '--ros-args', '--params-file', params_file],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'Septentrio GPS driver started (pid={self._gnss_proc.pid})')
        except Exception as e:
            self.get_logger().error(f'Failed to start GPS driver: {e}')

    def _gps_cb(self, msg):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        self.get_logger().info(
            f'GPS fix: lat={msg.latitude:.6f} lon={msg.longitude:.6f} '
            f'alt={msg.altitude:.1f}m status={msg.status.status}'
        )

    def destroy_node(self):
        if self._gnss_proc is not None:
            try:
                self._gnss_proc.send_signal(signal.SIGINT)
                self._gnss_proc.wait(timeout=5)
            except Exception:
                self._gnss_proc.kill()
            self.get_logger().info('GPS driver stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisherNode()
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
