#!/usr/bin/env python3
import os
import time
import json
import struct
import socket
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, Joy
from std_msgs.msg import Int32, String, UInt32
from std_srvs.srv import Trigger
from exia_msgs.msg import NavigationGoal

TCP_PORT = 5800
RECV_BUF = 65536
RECONNECT_INTERVAL = 2.0
HEARTBEAT_RATE = 10.0
HEARTBEAT_TIMEOUT = 1.5
STATUS_RATE = 5.0


class WifiBridge(Node):

    def __init__(self):
        super().__init__('wifi_bridge')

        self.declare_parameter('role', 'robot')
        self.declare_parameter('peer_ip', '192.168.100.168')
        self.declare_parameter('tcp_port', TCP_PORT)
        self.declare_parameter('heartbeat_rate', HEARTBEAT_RATE)
        self.declare_parameter('heartbeat_timeout', HEARTBEAT_TIMEOUT)
        self.declare_parameter('status_rate', STATUS_RATE)

        self._role = self.get_parameter('role').value
        self._peer_ip = self.get_parameter('peer_ip').value
        self._tcp_port = self.get_parameter('tcp_port').value
        self._heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self._heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self._status_rate = self.get_parameter('status_rate').value

        self._sock = None
        self._sock_lock = threading.Lock()
        self._recv_buf = b''
        self._connected = False
        self._last_reconnect = 0.0

        self._heartbeat_seq = 0
        self._last_heartbeat_rx = None
        self._heartbeat_ack_miss = 0
        self._estop_active = False
        self._estop_from_remote = False
        self._state_lock = threading.Lock()

        self._last_odom_x = 0.0
        self._last_odom_y = 0.0
        self._last_odom_heading = 0.0
        self._last_odom_speed = 0.0
        self._last_gps_lat = float('nan')
        self._last_gps_lon = float('nan')
        self._last_imu_heading = float('nan')
        self._last_imu_roll = float('nan')
        self._last_imu_pitch = float('nan')

        self._rx_count = 0
        self._tx_count = 0
        self._drop_count = 0
        self._last_log_time = 0.0

        self._cb_group = ReentrantCallbackGroup()

        if self._role == 'base':
            self._setup_base()
        else:
            self._setup_robot()

        self._net_thread = threading.Thread(target=self._net_loop, daemon=True)
        self._net_thread.start()

        self.create_timer(0.01, self._recv_timer, callback_group=self._cb_group)
        self.create_timer(5.0, self._log_stats, callback_group=self._cb_group)

        self.get_logger().info(
            f'WifiBridge started: role={self._role} peer={self._peer_ip}:{self._tcp_port}')

    def _send_msg(self, msg_type, payload=''):
        data = json.dumps({'t': msg_type, 'p': payload}).encode('utf-8')
        frame = struct.pack('>I', len(data)) + data
        with self._sock_lock:
            if self._sock is None:
                return False
            try:
                self._sock.sendall(frame)
                self._tx_count += 1
                return True
            except Exception:
                self._connected = False
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
                return False

    def _net_loop(self):
        if self._role == 'robot':
            self._run_server()
        else:
            self._run_client()

    def _run_server(self):
        while rclpy.ok():
            try:
                srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.settimeout(2.0)
                srv.bind(('0.0.0.0', self._tcp_port))
                srv.listen(1)
                self.get_logger().info(f'Listening on :{self._tcp_port}')
                while rclpy.ok():
                    try:
                        conn, addr = srv.accept()
                        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        conn.settimeout(0.01)
                        with self._sock_lock:
                            if self._sock is not None:
                                try:
                                    self._sock.close()
                                except Exception:
                                    pass
                            self._sock = conn
                            self._connected = True
                            self._recv_buf = b''
                        self._last_heartbeat_rx = time.monotonic()
                        with self._state_lock:
                            self._estop_active = False
                            self._estop_from_remote = False
                        self.get_logger().info(f'Base connected from {addr}')
                    except socket.timeout:
                        continue
            except Exception as e:
                self.get_logger().error(f'Server error: {e}')
                time.sleep(RECONNECT_INTERVAL)

    def _run_client(self):
        while rclpy.ok():
            if self._connected:
                time.sleep(0.5)
                continue
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self._peer_ip, self._tcp_port))
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(0.01)
                with self._sock_lock:
                    self._sock = s
                    self._connected = True
                    self._recv_buf = b''
                self.get_logger().info(f'Connected to {self._peer_ip}:{self._tcp_port}')
            except Exception:
                time.sleep(RECONNECT_INTERVAL)

    def _recv_timer(self):
        with self._sock_lock:
            if self._sock is None:
                return
            try:
                data = self._sock.recv(RECV_BUF)
                if not data:
                    self._connected = False
                    self._sock.close()
                    self._sock = None
                    self.get_logger().warn('Connection closed by peer')
                    return
                self._recv_buf += data
            except socket.timeout:
                pass
            except Exception:
                self._connected = False
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
                self.get_logger().warn('Connection lost')
                return

        while len(self._recv_buf) >= 4:
            msg_len = struct.unpack('>I', self._recv_buf[:4])[0]
            if msg_len > RECV_BUF:
                self._recv_buf = b''
                return
            if len(self._recv_buf) < 4 + msg_len:
                return
            raw = self._recv_buf[4:4 + msg_len]
            self._recv_buf = self._recv_buf[4 + msg_len:]
            try:
                msg = json.loads(raw)
                self._rx_count += 1
                if self._role == 'robot':
                    self._robot_dispatch(msg['t'], msg.get('p', ''))
                else:
                    self._base_dispatch(msg['t'], msg.get('p', ''))
            except Exception:
                self._drop_count += 1

    def _log_stats(self):
        now = time.monotonic()
        connected = self._connected
        self.get_logger().info(
            f'[WiFi] connected={connected} tx={self._tx_count} rx={self._rx_count} '
            f'drops={self._drop_count}')

    def _setup_base(self):
        self._goal_sub = self.create_subscription(
            NavigationGoal, '/navigation/goal',
            self._base_goal_cb, 10, callback_group=self._cb_group)

        self._status_pub = self.create_publisher(String, '/navigation/status', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._gps_pub = self.create_publisher(NavSatFix, '/navsatfix', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        self._cancel_srv = self.create_service(
            Trigger, '/radio/cancel', self._base_cancel_cb,
            callback_group=self._cb_group)
        self._estop_srv = self.create_service(
            Trigger, '/radio/estop', self._base_estop_cb,
            callback_group=self._cb_group)
        self._estop_clear_srv = self.create_service(
            Trigger, '/radio/estop_clear', self._base_estop_clear_cb,
            callback_group=self._cb_group)

        self._cmd_vel_sub = self.create_subscription(
            Twist, '/radio/cmd_vel', self._base_cmd_vel_cb, 10,
            callback_group=self._cb_group)
        self._last_cmd_vel_time = 0.0

        self._teleop_sub = self.create_subscription(
            Twist, '/radio/teleop', self._base_teleop_cb, 10,
            callback_group=self._cb_group)
        self._last_teleop_time = 0.0

        self._gear_sub = self.create_subscription(
            Int32, '/radio/gear', self._base_gear_cb, 10,
            callback_group=self._cb_group)

        self._heartbeat_timer = self.create_timer(
            1.0 / self._heartbeat_rate, self._base_send_heartbeat,
            callback_group=self._cb_group)

    def _base_goal_cb(self, msg):
        if not self._connected:
            return
        payload = {
            'coord_type': msg.coord_type,
            'x': msg.x, 'y': msg.y,
            'lat': msg.lat, 'lon': msg.lon,
            'lat_dms': msg.lat_dms, 'lon_dms': msg.lon_dms,
            'direct': msg.direct,
            'origin_lat': msg.origin_lat, 'origin_lon': msg.origin_lon,
            'move_type': msg.move_type,
            'move_value': msg.move_value, 'move_speed': msg.move_speed,
        }
        self._send_msg('N', payload)

    def _base_cancel_cb(self, request, response):
        if self._connected:
            self._send_msg('C')
            response.success = True
            response.message = 'Cancel sent over wifi'
        else:
            response.success = False
            response.message = 'Not connected'
        return response

    def _base_estop_cb(self, request, response):
        if self._connected:
            self._send_msg('E')
            response.success = True
            response.message = 'E-stop sent over wifi'
        else:
            response.success = False
            response.message = 'Not connected'
        return response

    def _base_estop_clear_cb(self, request, response):
        if self._connected:
            self._send_msg('X')
            response.success = True
            response.message = 'E-stop clear sent over wifi'
        else:
            response.success = False
            response.message = 'Not connected'
        return response

    def _base_cmd_vel_cb(self, msg):
        now = time.monotonic()
        if now - self._last_cmd_vel_time < 0.05:
            return
        self._last_cmd_vel_time = now
        self._send_msg('V', {'lx': msg.linear.x, 'az': msg.angular.z})

    def _base_teleop_cb(self, msg):
        now = time.monotonic()
        if now - self._last_teleop_time < 0.02:
            return
        self._last_teleop_time = now
        self._send_msg('D', {
            'lx': msg.linear.x, 'ly': msg.linear.y, 'az': msg.angular.z})

    def _base_gear_cb(self, msg):
        self._send_msg('G', msg.data)

    def _base_send_heartbeat(self):
        if not self._connected:
            return
        self._heartbeat_seq += 1
        self._send_msg('H', self._heartbeat_seq)

    def _base_dispatch(self, msg_type, payload):
        if msg_type == 'A':
            pass
        elif msg_type == 'S':
            self._base_handle_telemetry(payload)
        elif msg_type == 'R':
            msg = String()
            msg.data = payload
            self._status_pub.publish(msg)
        elif msg_type == 'EA':
            self.get_logger().info(f'E-stop ack from robot: {payload}')

    @staticmethod
    def _euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
        from geometry_msgs.msg import Quaternion
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    @staticmethod
    def _yaw_to_quaternion(yaw_deg):
        from geometry_msgs.msg import Quaternion
        yaw = math.radians(yaw_deg)
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def _base_handle_telemetry(self, payload):
        if not isinstance(payload, dict):
            return
        now_stamp = self.get_clock().now().to_msg()

        x = payload.get('x', 0.0)
        y = payload.get('y', 0.0)
        hdg = payload.get('hdg', 0.0)
        spd = payload.get('spd', 0.0)

        odom = Odometry()
        odom.header.stamp = now_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = self._yaw_to_quaternion(hdg)
        yaw_rad = math.radians(hdg)
        odom.twist.twist.linear.x = spd * math.cos(yaw_rad)
        odom.twist.twist.linear.y = spd * math.sin(yaw_rad)
        self._odom_pub.publish(odom)

        lat = payload.get('lat', float('nan'))
        lon = payload.get('lon', float('nan'))
        if math.isfinite(lat) and math.isfinite(lon):
            gps = NavSatFix()
            gps.header.stamp = now_stamp
            gps.header.frame_id = 'gps_link'
            gps.latitude = lat
            gps.longitude = lon
            gps.status.status = NavSatStatus.STATUS_FIX
            gps.status.service = NavSatStatus.SERVICE_GPS
            self._gps_pub.publish(gps)

        imu_h = payload.get('imu_h', float('nan'))
        imu_r = payload.get('imu_r', float('nan'))
        imu_p = payload.get('imu_p', float('nan'))
        if math.isfinite(imu_h):
            imu = Imu()
            imu.header.stamp = now_stamp
            imu.header.frame_id = 'imu_link'
            imu.orientation = self._euler_to_quaternion(imu_r, imu_p, imu_h)
            imu.orientation_covariance[0] = 0.01
            imu.orientation_covariance[4] = 0.01
            imu.orientation_covariance[8] = 0.01
            self._imu_pub.publish(imu)

    def _setup_robot(self):
        self._goal_pub = self.create_publisher(NavigationGoal, '/navigation/goal', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._teleop_pub = self.create_publisher(Twist, '/radio/teleop', 10)
        self._gear_pub = self.create_publisher(Int32, '/radio/gear', 10)

        self._cancel_client = self.create_client(
            Trigger, '/navigation/cancel', callback_group=self._cb_group)

        self._status_sub = self.create_subscription(
            String, '/navigation/status',
            self._robot_status_cb, 10, callback_group=self._cb_group)
        self._last_relayed_status = None
        self._last_relayed_status_time = 0.0

        self._gps_sub = self.create_subscription(
            NavSatFix, '/navsatfix', self._robot_gps_cb, 10,
            callback_group=self._cb_group)
        self._imu_sub = self.create_subscription(
            Imu, '/imu/data', self._robot_imu_cb, 10,
            callback_group=self._cb_group)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._robot_odom_cb, 10,
            callback_group=self._cb_group)

        self._watchdog_timer = self.create_timer(
            0.05, self._robot_watchdog, callback_group=self._cb_group)
        self._telemetry_timer = self.create_timer(
            1.0 / self._status_rate, self._robot_send_telemetry,
            callback_group=self._cb_group)

    def _robot_dispatch(self, msg_type, payload):
        if msg_type == 'H':
            self._robot_handle_heartbeat(payload)
        elif msg_type == 'N':
            self._robot_handle_nav_goal(payload)
        elif msg_type == 'C':
            self._robot_handle_cancel()
        elif msg_type == 'E':
            self._robot_handle_estop()
        elif msg_type == 'X':
            self._robot_handle_estop_clear()
        elif msg_type == 'V':
            self._robot_handle_cmd_vel(payload)
        elif msg_type == 'D':
            self._robot_handle_teleop(payload)
        elif msg_type == 'G':
            self._robot_handle_gear(payload)

    def _robot_handle_heartbeat(self, payload):
        self._last_heartbeat_rx = time.monotonic()
        auto_clear = False
        with self._state_lock:
            if self._estop_active and not self._estop_from_remote:
                self._estop_active = False
                auto_clear = True
        if auto_clear:
            self.get_logger().info('Heartbeat-loss e-stop auto-cleared')
            self._send_msg('EA', 'auto_cleared')
        self._send_msg('A', payload)

    def _robot_handle_nav_goal(self, payload):
        if not isinstance(payload, dict):
            return
        msg = NavigationGoal()
        msg.coord_type = payload.get('coord_type', '')
        msg.x = float(payload.get('x', 0.0))
        msg.y = float(payload.get('y', 0.0))
        msg.lat = float(payload.get('lat', 0.0))
        msg.lon = float(payload.get('lon', 0.0))
        msg.lat_dms = payload.get('lat_dms', '')
        msg.lon_dms = payload.get('lon_dms', '')
        msg.direct = bool(payload.get('direct', False))
        msg.origin_lat = float(payload.get('origin_lat', 0.0))
        msg.origin_lon = float(payload.get('origin_lon', 0.0))
        msg.move_type = payload.get('move_type', '')
        msg.move_value = float(payload.get('move_value', 0.0))
        msg.move_speed = float(payload.get('move_speed', 0.0))
        self._goal_pub.publish(msg)
        self.get_logger().info(f'Published nav goal: {msg.coord_type}')

    def _robot_handle_cancel(self):
        if not self._cancel_client.service_is_ready():
            self.get_logger().warn('Cancel service not ready')
            return
        future = self._cancel_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Cancel result: {f.result()}'))

    def _robot_handle_estop(self):
        with self._state_lock:
            self._estop_active = True
            self._estop_from_remote = True
        stop = Twist()
        self._cmd_vel_pub.publish(stop)
        teleop_stop = Twist()
        teleop_stop.linear.y = 1.0
        self._teleop_pub.publish(teleop_stop)
        gear_neutral = Int32()
        gear_neutral.data = 1
        self._gear_pub.publish(gear_neutral)
        self.get_logger().warn('Remote e-stop activated')
        self._send_msg('EA', 'activated')
        self._robot_handle_cancel()

    def _robot_handle_estop_clear(self):
        with self._state_lock:
            self._estop_active = False
            self._estop_from_remote = False
        self.get_logger().info('Remote e-stop cleared')
        self._send_msg('EA', 'cleared')

    def _robot_handle_cmd_vel(self, payload):
        if not isinstance(payload, dict):
            return
        with self._state_lock:
            if self._estop_active:
                return
        twist = Twist()
        twist.linear.x = float(payload.get('lx', 0.0))
        twist.angular.z = float(payload.get('az', 0.0))
        self._cmd_vel_pub.publish(twist)

    def _robot_handle_teleop(self, payload):
        if not isinstance(payload, dict):
            return
        with self._state_lock:
            if self._estop_active:
                stop = Twist()
                stop.linear.y = 1.0
                self._teleop_pub.publish(stop)
                return
        msg = Twist()
        msg.linear.x = float(payload.get('lx', 0.0))
        msg.linear.y = float(payload.get('ly', 0.0))
        msg.angular.z = float(payload.get('az', 0.0))
        self._teleop_pub.publish(msg)

    def _robot_handle_gear(self, payload):
        with self._state_lock:
            if self._estop_active:
                msg = Int32()
                msg.data = 1
                self._gear_pub.publish(msg)
                return
        msg = Int32()
        msg.data = int(payload)
        self._gear_pub.publish(msg)

    def _robot_watchdog(self):
        with self._state_lock:
            estop = self._estop_active
        if estop:
            stop = Twist()
            self._cmd_vel_pub.publish(stop)
            teleop_stop = Twist()
            teleop_stop.linear.y = 1.0
            self._teleop_pub.publish(teleop_stop)
        if self._last_heartbeat_rx is None:
            return
        elapsed = time.monotonic() - self._last_heartbeat_rx
        if elapsed > self._heartbeat_timeout:
            with self._state_lock:
                if not self._estop_active:
                    self._estop_active = True
                    self.get_logger().warn(
                        f'Heartbeat lost ({elapsed:.2f}s), e-stop')
                    stop = Twist()
                    self._cmd_vel_pub.publish(stop)
                    self._send_msg('EA', 'heartbeat_loss')
                    self._robot_handle_cancel()

    def _robot_gps_cb(self, msg):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self._last_gps_lat = msg.latitude
            self._last_gps_lon = msg.longitude

    def _robot_imu_cb(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._last_imu_heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._last_imu_roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._last_imu_pitch = math.degrees(math.asin(sinp))

    def _robot_odom_cb(self, msg):
        self._last_odom_x = msg.pose.pose.position.x
        self._last_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._last_odom_heading = math.degrees(math.atan2(siny, cosy))
        self._last_odom_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _robot_status_cb(self, msg):
        if not self._connected:
            return
        now = time.monotonic()
        if msg.data == self._last_relayed_status and now - self._last_relayed_status_time < 1.0:
            return
        self._last_relayed_status = msg.data
        self._last_relayed_status_time = now
        self._send_msg('R', msg.data)

    def _robot_send_telemetry(self):
        if not self._connected:
            return
        with self._state_lock:
            estop = self._estop_active
        self._send_msg('S', {
            'state': 'ESTOP' if estop else 'OK',
            'x': round(self._last_odom_x, 2),
            'y': round(self._last_odom_y, 2),
            'hdg': round(self._last_odom_heading, 1),
            'spd': round(self._last_odom_speed, 2),
            'lat': round(self._last_gps_lat, 8),
            'lon': round(self._last_gps_lon, 8),
            'imu_h': round(self._last_imu_heading, 1),
            'imu_r': round(self._last_imu_roll, 1),
            'imu_p': round(self._last_imu_pitch, 1),
        })

    def destroy_node(self):
        with self._sock_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WifiBridge()
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
