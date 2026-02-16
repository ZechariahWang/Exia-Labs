#!/usr/bin/env python3
import os
import time
import base64
import struct
import hashlib
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from exia_msgs.msg import NavigationGoal

import serial
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

SERIAL_BUFFER_MAX = 4096
SERIAL_RECONNECT_INTERVAL = 1.0
KEY_FRAGMENT_TIMEOUT = 0.5
HEARTBEAT_ACK_MISS_LIMIT = 15


def load_rsa_keys(key_dir):
    key_dir = os.path.expanduser(key_dir)
    private_key = None
    public_key = None

    priv_path = os.path.join(key_dir, 'radio_private.pem')
    pub_path = os.path.join(key_dir, 'radio_public.pem')

    if os.path.exists(priv_path):
        with open(priv_path, 'rb') as f:
            private_key = serialization.load_pem_private_key(f.read(), password=None)

    if os.path.exists(pub_path):
        with open(pub_path, 'rb') as f:
            public_key = serialization.load_pem_public_key(f.read())

    return private_key, public_key


def rsa_encrypt_aes_key(public_key, aes_key):
    return public_key.encrypt(
        aes_key,
        padding.OAEP(
            mgf=padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=None,
        ),
    )


def rsa_decrypt_aes_key(private_key, encrypted):
    return private_key.decrypt(
        encrypted,
        padding.OAEP(
            mgf=padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=None,
        ),
    )


def aes_key_hash(aes_key):
    return hashlib.sha256(aes_key).hexdigest()[:8]


def aes_encrypt(aes_key, counter, plaintext):
    nonce = struct.pack('<Q', counter) + b'\x00\x00\x00\x00'
    aesgcm = AESGCM(aes_key)
    ct = aesgcm.encrypt(nonce, plaintext.encode('utf-8'), None)
    return nonce + ct


def aes_decrypt(aes_key, data):
    if len(data) < 28:
        return None
    try:
        nonce = data[:12]
        ct = data[12:]
        aesgcm = AESGCM(aes_key)
        plaintext = aesgcm.decrypt(nonce, ct, None)
        return nonce, plaintext.decode('utf-8')
    except Exception:
        return None


def extract_nonce_counter(nonce):
    return struct.unpack('<Q', nonce[:8])[0]


def encode_plaintext(msg_type, payload):
    if payload:
        return f'{msg_type},{payload}'
    return msg_type


def decode_plaintext(text):
    if not text:
        return None
    comma = text.find(',')
    if comma == -1:
        return (text, '')
    return (text[:comma], text[comma + 1:])


def encode_frame(msg_type, payload, aes_key, counter):
    plaintext = encode_plaintext(msg_type, payload)
    encrypted = aes_encrypt(aes_key, counter, plaintext)
    return b'$' + base64.b64encode(encrypted) + b'\n'


def decode_frame(line, aes_key):
    if not line.startswith('$'):
        return None
    try:
        data = base64.b64decode(line[1:])
    except Exception:
        return None
    result = aes_decrypt(aes_key, data)
    if result is None:
        return None
    nonce, plaintext = result
    parsed = decode_plaintext(plaintext)
    if parsed is None:
        return None
    return (parsed[0], parsed[1], extract_nonce_counter(nonce))


class RadioBridge(Node):
    def __init__(self):
        super().__init__('radio_bridge')

        self.declare_parameter('role', 'robot')
        self.declare_parameter('serial_port', '/dev/exia_radio')
        self.declare_parameter('serial_baud', 57600)
        self.declare_parameter('heartbeat_rate', 10.0)
        self.declare_parameter('heartbeat_timeout', 0.5)
        self.declare_parameter('status_rate', 5.0)
        self.declare_parameter('key_dir', '~/.exia')

        self._role = self.get_parameter('role').value
        self._serial_port = self.get_parameter('serial_port').value
        self._serial_baud = self.get_parameter('serial_baud').value
        self._heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self._heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self._status_rate = self.get_parameter('status_rate').value
        self._key_dir = self.get_parameter('key_dir').value

        self._serial = None
        self._serial_lock = threading.Lock()
        self._serial_buffer = ''
        self._last_reconnect_attempt = 0.0

        self._state_lock = threading.Lock()
        self._aes_key = None
        self._tx_counter = 0 if self._role == 'base' else 1
        self._rx_expected_counter = 1 if self._role == 'base' else 0
        self._handshake_complete = False
        self._estop_active = False
        self._estop_from_remote = False

        self._rsa_private, self._rsa_public = load_rsa_keys(self._key_dir)

        self._heartbeat_seq = 0
        self._last_heartbeat_time = None
        self._heartbeat_ack_miss_count = 0
        self._last_heartbeat_ack_seq = 0

        self._key_fragment_1 = None
        self._key_fragment_1_time = 0.0

        self._last_odom_x = 0.0
        self._last_odom_y = 0.0
        self._last_odom_heading = 0.0
        self._last_odom_speed = 0.0

        self._last_telemetry_info_time = 0.0

        self._last_sent_goal_hash = None
        self._last_sent_goal_time = 0.0
        self._recent_local_goals = []
        self._last_relayed_status = None
        self._last_relayed_status_time = 0.0

        self._cb_group = ReentrantCallbackGroup()

        self._open_serial()

        self._read_timer = self.create_timer(
            0.01, self._serial_read_timer, callback_group=self._cb_group
        )

        if self._role == 'base':
            self._setup_base()
        else:
            self._setup_robot()

        self.get_logger().info(f'RadioBridge started: role={self._role}, port={self._serial_port}')

    def _reset_handshake_state(self):
        with self._state_lock:
            self._handshake_complete = False
            self._aes_key = None
            self._tx_counter = 0 if self._role == 'base' else 1
            self._rx_expected_counter = 1 if self._role == 'base' else 0
            self._heartbeat_ack_miss_count = 0
            self._last_heartbeat_ack_seq = 0
            self._key_fragment_1 = None
            self._key_fragment_1_time = 0.0
            if self._role == 'base':
                self._aes_key = os.urandom(32)

    def _open_serial(self):
        self._reset_handshake_state()
        try:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._serial_baud,
                timeout=0.01,
                rtscts=False,
                dsrdtr=False,
                write_timeout=0.1,
            )
            time.sleep(3.0)
            self._serial.reset_input_buffer()
            self._serial_buffer = ''
            self.get_logger().info(f'Serial connected: {self._serial_port}')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self._serial = None

    def _serial_write_raw(self, data):
        if self._serial is None:
            return False
        try:
            with self._serial_lock:
                self._serial.write(data)
                self._serial.flush()
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')
            self._serial = None
            return False
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')
            return False

    def _serial_write(self, msg_type, payload=''):
        with self._state_lock:
            if not self._handshake_complete:
                return False
            frame = encode_frame(msg_type, payload, self._aes_key, self._tx_counter)
            self._tx_counter += 2
        with self._serial_lock:
            if self._serial is None:
                return False
            try:
                self._serial.write(frame)
                self._serial.flush()
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial write error: {e}')
                self._serial = None
                return False
            except Exception as e:
                self.get_logger().warn(f'Serial write error: {e}')
                return False
        return True

    def _serial_read_timer(self):
        if self._serial is None:
            now = time.monotonic()
            if now - self._last_reconnect_attempt >= SERIAL_RECONNECT_INTERVAL:
                self._last_reconnect_attempt = now
                self.get_logger().info('Attempting serial reconnect...')
                self._open_serial()
            return

        if not os.path.exists(self._serial_port):
            self.get_logger().warn('Serial device disconnected')
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            return

        try:
            with self._serial_lock:
                waiting = self._serial.in_waiting
                if waiting > 0:
                    data = self._serial.read(waiting).decode('utf-8', errors='ignore')
                    self._serial_buffer += data
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            self._serial = None
            return
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if '\n' not in self._serial_buffer and len(self._serial_buffer) > SERIAL_BUFFER_MAX:
            self.get_logger().warn(f'Serial buffer overflow ({len(self._serial_buffer)} bytes), trimming')
            last_marker = self._serial_buffer.rfind('$')
            if last_marker >= 0:
                self._serial_buffer = self._serial_buffer[last_marker:]
            else:
                self._serial_buffer = ''
            return

        while '\n' in self._serial_buffer:
            line, self._serial_buffer = self._serial_buffer.split('\n', 1)
            line = line.strip()
            if not line:
                continue
            self._dispatch_line(line)

    def _dispatch_line(self, line):
        if line.startswith('$K1,'):
            self._handle_key_fragment_1(line)
            return

        if line.startswith('$K2,'):
            self._handle_key_fragment_2(line)
            return

        if line.startswith('$A,'):
            self._handle_key_ack(line)
            return

        with self._state_lock:
            if not self._handshake_complete or self._aes_key is None:
                self.get_logger().debug(f'Ignoring line (no handshake): {line[:40]}')
                return
            aes_key = self._aes_key

        result = decode_frame(line, aes_key)
        if result is None:
            self.get_logger().warn(f'Failed to decode frame: len={len(line)} start={line[:30]}')
            return

        msg_type, payload, nonce_counter = result

        with self._state_lock:
            if nonce_counter < self._rx_expected_counter:
                self.get_logger().warn(f'Stale nonce {nonce_counter} < {self._rx_expected_counter}, dropping')
                return
            self._rx_expected_counter = nonce_counter + 2

        if self._role == 'robot':
            self._robot_dispatch(msg_type, payload)
        else:
            self._base_dispatch(msg_type, payload)

    def _setup_base(self):
        with self._state_lock:
            self._aes_key = os.urandom(32)

        self._goal_sub = self.create_subscription(
            NavigationGoal, '/navigation/goal',
            self._base_goal_callback, 10,
            callback_group=self._cb_group,
        )

        self._status_pub = self.create_publisher(String, '/navigation/status', 10)

        self._cancel_srv = self.create_service(
            Trigger, '/radio/cancel',
            self._base_cancel_callback,
            callback_group=self._cb_group,
        )

        self._estop_srv = self.create_service(
            Trigger, '/radio/estop',
            self._base_estop_callback,
            callback_group=self._cb_group,
        )

        self._estop_clear_srv = self.create_service(
            Trigger, '/radio/estop_clear',
            self._base_estop_clear_callback,
            callback_group=self._cb_group,
        )

        self._heartbeat_timer = self.create_timer(
            1.0 / self._heartbeat_rate,
            self._base_send_heartbeat,
            callback_group=self._cb_group,
        )

        self.create_timer(1.0, self._base_try_handshake, callback_group=self._cb_group)

    def _base_try_handshake(self):
        with self._state_lock:
            if self._handshake_complete:
                return
        if self._serial is None:
            return
        if self._rsa_public is None:
            self.get_logger().error('No RSA public key found')
            return

        with self._state_lock:
            aes_key = self._aes_key

        encrypted = rsa_encrypt_aes_key(self._rsa_public, aes_key)
        b64 = base64.b64encode(encrypted).decode('ascii')
        mid = len(b64) // 2
        frame1 = f'$K1,{b64[:mid]}\n'.encode('utf-8')
        frame2 = f'$K2,{b64[mid:]}\n'.encode('utf-8')
        self._serial_write_raw(frame1)
        time.sleep(0.05)
        self._serial_write_raw(frame2)
        self.get_logger().info('Sent fragmented key exchange')

    def _handle_key_fragment_1(self, line):
        if self._role != 'robot':
            return
        with self._state_lock:
            self._key_fragment_1 = line[4:]
            self._key_fragment_1_time = time.monotonic()

    def _handle_key_fragment_2(self, line):
        if self._role != 'robot':
            return
        if self._rsa_private is None:
            self.get_logger().error('No RSA private key found')
            return

        with self._state_lock:
            if self._key_fragment_1 is None:
                self.get_logger().warn('Received K2 without K1, discarding')
                return
            elapsed = time.monotonic() - self._key_fragment_1_time
            if elapsed > KEY_FRAGMENT_TIMEOUT:
                self.get_logger().warn(f'K1 fragment expired ({elapsed:.2f}s), discarding')
                self._key_fragment_1 = None
                return
            b64_data = self._key_fragment_1 + line[4:]
            self._key_fragment_1 = None

        try:
            encrypted = base64.b64decode(b64_data)
            aes_key = rsa_decrypt_aes_key(self._rsa_private, encrypted)
            with self._state_lock:
                self._aes_key = aes_key
                self._handshake_complete = True
                self._tx_counter = 1
                self._rx_expected_counter = 0
            key_hash = aes_key_hash(aes_key)
            ack = f'$A,{key_hash}\n'.encode('utf-8')
            self._serial_write_raw(ack)
            self.get_logger().info('Key exchange complete (robot)')
        except Exception as e:
            self.get_logger().error(f'Key exchange failed: {e}')

    def _handle_key_ack(self, line):
        if self._role != 'base':
            return

        received_hash = line[3:].strip()
        with self._state_lock:
            expected_hash = aes_key_hash(self._aes_key)

        if received_hash != expected_hash:
            self.get_logger().error(f'Key ACK hash mismatch: got {received_hash}, expected {expected_hash}')
            return

        with self._state_lock:
            self._handshake_complete = True
            self._heartbeat_ack_miss_count = 0
        self.get_logger().info('Key exchange complete (base)')

    def _goal_hash(self, msg):
        return (
            msg.coord_type, round(msg.x, 4), round(msg.y, 4),
            round(msg.lat, 8), round(msg.lon, 8),
            msg.lat_dms, msg.lon_dms, msg.direct,
        )

    def _base_goal_callback(self, msg):
        with self._state_lock:
            if not self._handshake_complete:
                self.get_logger().warn('Cannot send goal: handshake not complete')
                return

        goal_hash = self._goal_hash(msg)
        now = time.monotonic()
        if goal_hash == self._last_sent_goal_hash and now - self._last_sent_goal_time < 2.0:
            return

        if msg.coord_type == 'xy':
            payload = f'xy,{msg.x:.6f},{msg.y:.6f},{int(msg.direct)}'
            if msg.origin_lat != 0.0 or msg.origin_lon != 0.0:
                payload += f',{msg.origin_lat:.10f},{msg.origin_lon:.10f}'
        elif msg.coord_type == 'latlon':
            payload = f'latlon,{msg.lat:.10f},{msg.lon:.10f},{int(msg.direct)}'
            if msg.origin_lat != 0.0 or msg.origin_lon != 0.0:
                payload += f',{msg.origin_lat:.10f},{msg.origin_lon:.10f}'
        elif msg.coord_type == 'dms':
            payload = f'dms|{msg.lat_dms}|{msg.lon_dms}|{int(msg.direct)}'
            if msg.origin_lat != 0.0 or msg.origin_lon != 0.0:
                payload += f'|{msg.origin_lat:.10f}|{msg.origin_lon:.10f}'
        elif msg.coord_type == 'forward':
            payload = f'forward,{msg.move_type},{msg.move_value:.6f},{msg.move_speed:.6f}'
        elif msg.coord_type == 'turn':
            payload = f'turn,{msg.move_type},{msg.move_value:.6f},{msg.move_speed:.6f}'
        else:
            self.get_logger().warn(f'Unknown coord_type: {msg.coord_type}')
            return

        self._last_sent_goal_hash = goal_hash
        self._last_sent_goal_time = now
        self._serial_write('N', payload)
        self.get_logger().info(f'Sent nav goal: {msg.coord_type}')

    def _base_cancel_callback(self, request, response):
        with self._state_lock:
            handshake_ok = self._handshake_complete
        if handshake_ok:
            self._serial_write('C')
            response.success = True
            response.message = 'Cancel sent over radio'
        else:
            response.success = False
            response.message = 'Radio handshake not complete'
        return response

    def _base_estop_callback(self, request, response):
        with self._state_lock:
            handshake_ok = self._handshake_complete
        if handshake_ok:
            self._serial_write('E')
            response.success = True
            response.message = 'E-stop sent over radio'
        else:
            response.success = False
            response.message = 'Radio handshake not complete'
        return response

    def _base_estop_clear_callback(self, request, response):
        with self._state_lock:
            handshake_ok = self._handshake_complete
        if handshake_ok:
            self._serial_write('X')
            response.success = True
            response.message = 'E-stop clear sent over radio'
        else:
            response.success = False
            response.message = 'Radio handshake not complete'
        return response

    def _base_send_heartbeat(self):
        with self._state_lock:
            if not self._handshake_complete:
                return
            self._heartbeat_seq += 1
            seq = self._heartbeat_seq
            self._heartbeat_ack_miss_count += 1
            if self._heartbeat_ack_miss_count >= HEARTBEAT_ACK_MISS_LIMIT:
                self.get_logger().warn(f'No heartbeat ACK for {HEARTBEAT_ACK_MISS_LIMIT} beats, resetting handshake')
                self._handshake_complete = False
                self._aes_key = os.urandom(32)
                self._tx_counter = 0
                self._rx_expected_counter = 1
                self._heartbeat_ack_miss_count = 0
                return
        self._serial_write('H', str(seq))

    def _base_dispatch(self, msg_type, payload):
        if msg_type == 'A':
            with self._state_lock:
                self._heartbeat_ack_miss_count = 0
        elif msg_type == 'S':
            self._base_handle_telemetry(payload)
        elif msg_type == 'R':
            self._base_handle_relay(payload)
        elif msg_type == 'EA':
            self.get_logger().info('Received e-stop acknowledgment from robot')

    def _base_handle_telemetry(self, payload):
        parts = payload.split(',')
        if len(parts) >= 5:
            state, x, y, hdg, spd = parts[0], parts[1], parts[2], parts[3], parts[4]
            now = time.monotonic()
            if now - self._last_telemetry_info_time >= 5.0:
                self.get_logger().info(
                    f'[Telemetry] state={state} pos=({x},{y}) hdg={hdg} spd={spd}'
                )
                self._last_telemetry_info_time = now
            else:
                self.get_logger().debug(
                    f'[Telemetry] state={state} pos=({x},{y}) hdg={hdg} spd={spd}'
                )

    def _base_handle_relay(self, payload):
        msg = String()
        msg.data = payload
        self._status_pub.publish(msg)

    def _setup_robot(self):
        self._goal_pub = self.create_publisher(NavigationGoal, '/navigation/goal', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._cancel_client = self.create_client(
            Trigger, '/navigation/cancel', callback_group=self._cb_group,
        )

        self._goal_monitor_sub = self.create_subscription(
            NavigationGoal, '/navigation/goal',
            self._robot_local_goal_callback, 10,
            callback_group=self._cb_group,
        )

        self._status_sub = self.create_subscription(
            String, '/navigation/status',
            self._robot_status_callback, 10,
            callback_group=self._cb_group,
        )

        self._odom_sub = self.create_subscription(
            Odometry, '/odom',
            self._robot_odom_callback, 10,
            callback_group=self._cb_group,
        )

        self._watchdog_timer = self.create_timer(
            0.05, self._robot_watchdog, callback_group=self._cb_group,
        )

        self._telemetry_timer = self.create_timer(
            1.0 / self._status_rate,
            self._robot_send_telemetry,
            callback_group=self._cb_group,
        )

    def _robot_local_goal_callback(self, msg):
        h = self._goal_hash(msg)
        now = time.monotonic()
        self._recent_local_goals = [(gh, t) for gh, t in self._recent_local_goals if now - t < 2.0]
        self._recent_local_goals.append((h, now))

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

    def _robot_handle_heartbeat(self, payload):
        self._last_heartbeat_time = time.monotonic()
        auto_clear = False
        with self._state_lock:
            if self._estop_active and not self._estop_from_remote:
                self._estop_active = False
                auto_clear = True
        if auto_clear:
            self.get_logger().info('Heartbeat-loss e-stop auto-cleared')
            self._serial_write('EA', 'auto_cleared')
        self._serial_write('A', payload)

    def _robot_handle_nav_goal(self, payload):
        msg = self._parse_nav_payload(payload)
        if msg is None:
            return

        h = self._goal_hash(msg)
        now = time.monotonic()
        if any(gh == h and now - t < 2.0 for gh, t in self._recent_local_goals):
            return

        self._goal_pub.publish(msg)
        self.get_logger().info(f'Published nav goal: {msg.coord_type}')

    def _parse_nav_payload(self, payload):
        msg = NavigationGoal()

        if payload.startswith('dms|'):
            parts = payload.split('|')
            if len(parts) < 4:
                return None
            msg.coord_type = 'dms'
            try:
                msg.lat_dms = parts[1]
                msg.lon_dms = parts[2]
                msg.direct = bool(int(parts[3]))
                if len(parts) >= 6:
                    msg.origin_lat = float(parts[4])
                    msg.origin_lon = float(parts[5])
            except (IndexError, ValueError) as e:
                self.get_logger().warn(f'Failed to parse dms nav goal: {e}')
                return None
        else:
            parts = payload.split(',')
            if len(parts) < 2:
                return None
            coord_type = parts[0]
            msg.coord_type = coord_type
            try:
                if coord_type == 'xy':
                    msg.x = float(parts[1])
                    msg.y = float(parts[2])
                    msg.direct = bool(int(parts[3]))
                    if len(parts) >= 6:
                        msg.origin_lat = float(parts[4])
                        msg.origin_lon = float(parts[5])
                elif coord_type == 'latlon':
                    msg.lat = float(parts[1])
                    msg.lon = float(parts[2])
                    msg.direct = bool(int(parts[3]))
                    if len(parts) >= 6:
                        msg.origin_lat = float(parts[4])
                        msg.origin_lon = float(parts[5])
                elif coord_type == 'forward':
                    msg.move_type = parts[1]
                    msg.move_value = float(parts[2])
                    msg.move_speed = float(parts[3])
                elif coord_type == 'turn':
                    msg.move_type = parts[1]
                    msg.move_value = float(parts[2])
                    msg.move_speed = float(parts[3])
                else:
                    self.get_logger().warn(f'Unknown coord_type: {coord_type}')
                    return None
            except (IndexError, ValueError) as e:
                self.get_logger().warn(f'Failed to parse nav goal: {e}')
                return None

        return msg

    def _robot_handle_cancel(self):
        if not self._cancel_client.service_is_ready():
            self.get_logger().warn('Cancel service not ready')
            return
        future = self._cancel_client.call_async(Trigger.Request())
        future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f'Cancel result: {result.message}')
        except Exception as e:
            self.get_logger().warn(f'Cancel call failed: {e}')

    def _robot_handle_estop(self):
        with self._state_lock:
            self._estop_active = True
            self._estop_from_remote = True
        stop = Twist()
        self._cmd_vel_pub.publish(stop)
        self.get_logger().warn('Remote e-stop activated')
        self._serial_write('EA', 'activated')
        self._robot_handle_cancel()

    def _robot_handle_estop_clear(self):
        with self._state_lock:
            self._estop_active = False
            self._estop_from_remote = False
        self.get_logger().info('Remote e-stop cleared')
        self._serial_write('EA', 'cleared')

    def _robot_watchdog(self):
        with self._state_lock:
            estop = self._estop_active

        if estop:
            stop = Twist()
            self._cmd_vel_pub.publish(stop)

        if self._last_heartbeat_time is None:
            return

        elapsed = time.monotonic() - self._last_heartbeat_time
        if elapsed > self._heartbeat_timeout:
            with self._state_lock:
                already_estopped = self._estop_active
                if not already_estopped:
                    self._estop_active = True
            if not already_estopped:
                self.get_logger().warn(f'Radio heartbeat lost ({elapsed:.2f}s), e-stop')
                stop = Twist()
                self._cmd_vel_pub.publish(stop)
                self._serial_write('EA', 'heartbeat_loss')
                self._robot_handle_cancel()

    def _robot_odom_callback(self, msg):
        self._last_odom_x = msg.pose.pose.position.x
        self._last_odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._last_odom_heading = math.degrees(math.atan2(siny, cosy))

        self._last_odom_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        )

    def _robot_status_callback(self, msg):
        with self._state_lock:
            if not self._handshake_complete:
                return
        now = time.monotonic()
        if msg.data == self._last_relayed_status and now - self._last_relayed_status_time < 1.0:
            return
        self._last_relayed_status = msg.data
        self._last_relayed_status_time = now
        self._serial_write('R', msg.data)

    def _robot_send_telemetry(self):
        with self._state_lock:
            if not self._handshake_complete:
                return
            estop = self._estop_active
        state = 'ESTOP' if estop else 'OK'
        payload = (
            f'{state},{self._last_odom_x:.2f},{self._last_odom_y:.2f},'
            f'{self._last_odom_heading:.1f},{self._last_odom_speed:.2f}'
        )
        self._serial_write('S', payload)

    def destroy_node(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadioBridge()
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
