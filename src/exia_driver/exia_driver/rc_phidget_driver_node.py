#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import time
import math
import threading
import socket
import xml.etree.ElementTree as ET
from enum import IntEnum
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Optional

try:
    from Phidget22.Devices.MotorPositionController import MotorPositionController
    from Phidget22.PhidgetException import PhidgetException
    PHIDGETS_AVAILABLE = True
except ImportError:
    PHIDGETS_AVAILABLE = False


class State(IntEnum):
    IDLE = 0
    CONNECTING = 1
    CALIBRATING = 2
    ARMED = 3
    ESTOP = 4
    ERROR = 5


@dataclass
class PhidgetConfig:
    hub_port: int = 0
    kp: float = 1
    ki: float = 1
    kd: float = 1
    velocity_limit: float = 10000.0
    acceleration: float = 50000.0
    dead_band: float = 2.0
    current_limit: float = 15.0
    motor_degrees_at_max_steer: float = 2160.0
    failsafe_timeout: int = 1500


@dataclass
class SafetyConfig:
    max_position: float = 2160.0
    command_timeout: float = 1.0
    feedback_rate: float = 20.0
    control_rate: float = 50.0
    ramp_rate: float = 5.0
    smoothing_alpha: float = 0.1
    scurve_sharpness: float = 3.0
    runaway_duty_threshold: float = 0.95
    runaway_position_error: float = 100.0
    runaway_max_count: int = 25


@dataclass
class TakConfig:
    enabled: bool = False
    gps_topic: str = '/navsatfix'
    host: str = '192.168.1.69'
    port: int = 4242
    uid: str = 'exialabs-argus-1'
    callsign: str = 'exialabs-argus-1'
    cot_type: str = 'a-f-G-E-V-U'
    how: str = 'm-g'
    iconsetpath: str = 'COT_MAPPING_2525C'
    rate_hz: float = 0.2
    stale_seconds: float = 60.0
    default_hae: float = 0.0
    default_ce: float = 10.0
    default_le: float = 10.0
    require_fix: bool = True


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return False


class RCPhidgetDriverNode(Node):
    WHEELBASE = 1.3
    MAX_STEERING_ANGLE = 0.6
    MAX_SPEED = 5.0
    HW_THROTTLE_MAX = 75
    HW_BRAKE_FULL = 80
    HW_BRAKE_RELEASED = 180

    GEAR_SHIFT_IDLE = 0
    GEAR_SHIFT_BRAKING = 1
    GEAR_SHIFT_TO_NEUTRAL = 2
    GEAR_SHIFT_TO_TARGET = 3

    def __init__(self):
        super().__init__('rc_phidget_driver_node')

        self._declare_parameters()

        self.state = State.IDLE
        self.last_command_time = time.monotonic()
        self.last_feedback_time = time.monotonic()
        self.armed_time = 0.0
        self.startup_grace_period = 15.0
        self.current_target = 0.0
        self.smoothed_target = 0.0
        self.rc_lost = False
        self._timeout_warned = False

        self._phidgets_controller = None
        self._motor_ok = False
        self._motor_lock = threading.Lock()
        self._phidgets_runaway_count = 0

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.serial_conn = None
        self.serial_buffer = ""
        self.serial_lock = threading.Lock()

        self.phidget_config = PhidgetConfig(
            hub_port=self.get_parameter('phidgets_hub_port').value,
            kp=self.get_parameter('steering_kp').value,
            ki=self.get_parameter('steering_ki').value,
            kd=self.get_parameter('steering_kd').value,
            velocity_limit=self.get_parameter('steering_velocity_limit').value,
            acceleration=self.get_parameter('steering_acceleration').value,
            dead_band=self.get_parameter('steering_dead_band').value,
            current_limit=self.get_parameter('steering_current_limit').value,
            motor_degrees_at_max_steer=self.get_parameter('motor_degrees_at_max_steer').value,
            failsafe_timeout=self.get_parameter('failsafe_timeout').value,
        )

        self.safety_config = SafetyConfig(
            max_position=self.get_parameter('motor_degrees_at_max_steer').value,
            command_timeout=self.get_parameter('command_timeout').value,
            feedback_rate=self.get_parameter('feedback_rate').value,
            control_rate=self.get_parameter('control_rate').value,
            ramp_rate=self.get_parameter('ramp_rate').value,
            smoothing_alpha=self.get_parameter('smoothing_alpha').value,
            scurve_sharpness=self.get_parameter('scurve_sharpness').value,
        )

        self.tak_config = TakConfig(
            enabled=_as_bool(self.get_parameter('tak_udp_enabled').value),
            gps_topic=str(self.get_parameter('tak_gps_topic').value),
            host=str(self.get_parameter('tak_host').value),
            port=int(self.get_parameter('tak_port').value),
            uid=str(self.get_parameter('tak_uid').value),
            callsign=str(self.get_parameter('tak_callsign').value),
            cot_type=str(self.get_parameter('tak_type').value),
            how=str(self.get_parameter('tak_how').value),
            iconsetpath=str(self.get_parameter('tak_iconsetpath').value),
            rate_hz=float(self.get_parameter('tak_rate_hz').value),
            stale_seconds=float(self.get_parameter('tak_stale_seconds').value),
            default_hae=float(self.get_parameter('tak_default_hae').value),
            default_ce=float(self.get_parameter('tak_default_ce').value),
            default_le=float(self.get_parameter('tak_default_le').value),
            require_fix=_as_bool(self.get_parameter('tak_require_fix').value),
        )
        if self.tak_config.rate_hz <= 0.0:
            self.tak_config.rate_hz = 1.0
        self._tak_min_interval = 1.0 / self.tak_config.rate_hz
        self._tak_last_send_time = 0.0
        self._tak_last_warn_time = 0.0
        self._tak_socket = None
        self._gps_sub = None
        if self.tak_config.enabled:
            try:
                self._tak_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._tak_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self._gps_sub = self.create_subscription(
                    NavSatFix, self.tak_config.gps_topic, self._gps_callback, 10)
                self.get_logger().info(
                    f'TAK UDP enabled: topic={self.tak_config.gps_topic} -> '
                    f'{self.tak_config.host}:{self.tak_config.port} '
                    f'uid={self.tak_config.uid} callsign={self.tak_config.callsign} '
                    f'rate={self.tak_config.rate_hz:.2f}Hz'
                )
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize TAK UDP socket: {e}')
                self._tak_socket = None
                self.tak_config.enabled = False

        self.steering_pub = self.create_publisher(Float64, '/phidget/steering_position', 10)
        self.state_pub = self.create_publisher(String, '/phidget/state', 10)

        self.estop_srv = self.create_service(Trigger, '/phidget/emergency_stop', self._estop_callback)
        self.clear_srv = self.create_service(Trigger, '/phidget/clear_estop', self._clear_estop_callback)
        self.arm_srv = self.create_service(Trigger, '/phidget/arm', self._arm_callback)
        self.set_mode_srv = self.create_service(Trigger, '/driver/set_mode', self._set_mode_callback)

        self.autonomous_mode = self.get_parameter('autonomous_mode').value
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = time.monotonic()
        self.current_gear = 'neutral'
        self.autonomous_started = False

        self._gear_shift_state = self.GEAR_SHIFT_IDLE
        self._gear_shift_target = 'neutral'
        self._gear_shift_time = 0.0

        if self.autonomous_mode:
            self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
            self.auto_timer = self.create_timer(1.0 / 50.0, self._autonomous_control_loop)

        control_period = 1.0 / self.safety_config.control_rate
        feedback_period = 1.0 / self.safety_config.feedback_rate
        self.control_timer = self.create_timer(control_period, self._control_loop)
        self.feedback_timer = self.create_timer(feedback_period, self._feedback_loop)
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self._init_serial()
        if self.autonomous_mode and self.serial_conn is not None:
            time.sleep(0.5)
            self.serial_conn.reset_input_buffer()
            self._write_serial('AUTO')
            time.sleep(0.1)
            self.serial_conn.reset_input_buffer()
            self.get_logger().info('Sent AUTO to Arduino early (before Phidget connect)')
        self.autonomous_grace_time = time.monotonic()
        self._transition_state(State.CONNECTING)

        self.get_logger().info('RC Phidget Driver Node started')

    def _declare_parameters(self):
        self.declare_parameter('serial_port', '/dev/arduino_control')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('phidgets_hub_port', 0)
        self.declare_parameter('motor_degrees_at_max_steer', 2160.0)
        self.declare_parameter('steering_kp', 400.0)
        self.declare_parameter('steering_ki', 0.0)
        self.declare_parameter('steering_kd', 150.0)
        self.declare_parameter('steering_velocity_limit', 10000.0)
        self.declare_parameter('steering_acceleration', 50000.0)
        self.declare_parameter('steering_dead_band', 2.0)
        self.declare_parameter('steering_current_limit', 15.0)
        self.declare_parameter('failsafe_timeout', 1500)
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('feedback_rate', 20.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('ramp_rate', 5.0)
        self.declare_parameter('smoothing_alpha', 0.1)
        self.declare_parameter('scurve_sharpness', 3.0)
        self.declare_parameter('autonomous_mode', False)
        self.declare_parameter('tak_udp_enabled', False)
        self.declare_parameter('tak_gps_topic', '/navsatfix')
        self.declare_parameter('tak_host', '192.168.1.69')
        self.declare_parameter('tak_port', 4242)
        self.declare_parameter('tak_uid', 'exialabs-argus-1')
        self.declare_parameter('tak_callsign', 'exialabs-argus-1')
        self.declare_parameter('tak_type', 'a-f-G-E-V-U')
        self.declare_parameter('tak_how', 'm-g')
        self.declare_parameter('tak_iconsetpath', 'COT_MAPPING_2525C')
        self.declare_parameter('tak_rate_hz', 0.2)
        self.declare_parameter('tak_stale_seconds', 60.0)
        self.declare_parameter('tak_default_hae', 0.0)
        self.declare_parameter('tak_default_ce', 10.0)
        self.declare_parameter('tak_default_le', 10.0)
        self.declare_parameter('tak_require_fix', True)

    @staticmethod
    def _cot_time(dt: datetime) -> str:
        return dt.strftime('%Y-%m-%dT%H:%M:%S.%fZ')

    def _build_tak_cot(self, lat: float, lon: float, hae: float) -> bytes:
        now = datetime.now(timezone.utc)
        stale = now + timedelta(seconds=self.tak_config.stale_seconds)
        root = ET.Element(
            'event',
            {
                'version': '2.0',
                'uid': self.tak_config.uid,
                'type': self.tak_config.cot_type,
                'how': self.tak_config.how,
                'time': self._cot_time(now),
                'start': self._cot_time(now),
                'stale': self._cot_time(stale),
            },
        )
        ET.SubElement(
            root,
            'point',
            {
                'lat': f'{lat:.8f}',
                'lon': f'{lon:.8f}',
                'hae': f'{hae:.2f}',
                'ce': f'{self.tak_config.default_ce:.2f}',
                'le': f'{self.tak_config.default_le:.2f}',
            },
        )
        detail = ET.SubElement(root, 'detail')
        ET.SubElement(detail, 'contact', {'callsign': self.tak_config.callsign})
        ET.SubElement(detail, '__group', {'name': 'Blue', 'role': 'Team Member'})
        if self.tak_config.iconsetpath:
            ET.SubElement(detail, 'usericon', {'iconsetpath': self.tak_config.iconsetpath})
        return ET.tostring(root, encoding='utf-8')

    def _gps_callback(self, msg: NavSatFix):
        if not self.tak_config.enabled or self._tak_socket is None:
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        if self.tak_config.require_fix and msg.status.status < NavSatStatus.STATUS_FIX:
            return

        now = time.monotonic()
        if now - self._tak_last_send_time < self._tak_min_interval:
            return

        hae = msg.altitude if math.isfinite(msg.altitude) else self.tak_config.default_hae
        payload = self._build_tak_cot(msg.latitude, msg.longitude, hae)

        try:
            self._tak_socket.sendto(payload, (self.tak_config.host, self.tak_config.port))
            self._tak_last_send_time = now
        except Exception as e:
            if now - self._tak_last_warn_time > 2.0:
                self._tak_last_warn_time = now
                self.get_logger().warn(
                    f'TAK UDP send failed to {self.tak_config.host}:{self.tak_config.port}: {e}'
                )

    def _init_serial(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial connected: {self.serial_port}')
            time.sleep(1.0)
            self.serial_conn.reset_input_buffer()
            self.serial_conn.timeout = 0.01
            self.get_logger().info('Serial ready')
            return True
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.serial_conn = None
            return False

    def _read_serial(self) -> Optional[str]:
        if self.serial_conn is None:
            return None

        try:
            with self.serial_lock:
                waiting = self.serial_conn.in_waiting
                if waiting > 0:
                    data = self.serial_conn.read(waiting).decode('utf-8', errors='ignore')
                    self.serial_buffer += data

                if '\n' in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        return line
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
        return None

    def _write_serial(self, msg: str) -> bool:
        if self.serial_conn is None:
            return False

        try:
            with self.serial_lock:
                self.serial_conn.write((msg + '\n').encode('utf-8'))
                self.serial_conn.flush()
            return True
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')
            return False

    def _init_phidgets(self) -> bool:
        if not PHIDGETS_AVAILABLE:
            self.get_logger().error('Phidget22 library not available')
            return False

        try:
            ch = MotorPositionController()
            ch.setHubPort(self.phidget_config.hub_port)
            ch.setIsHubPortDevice(False)
            ch.setOnAttachHandler(self._on_attach)
            ch.setOnDetachHandler(self._on_detach)
            self._phidgets_controller = ch
            ch.open()
            self.get_logger().info('MotorPositionController channel opened, waiting for attach...')
            return True
        except PhidgetException as e:
            self.get_logger().error(f'Phidgets init failed: {e}')
            return False

    def _on_attach(self, sender):
        try:
            sender.setRescaleFactor(360.0 / (300 * 4 * 4.25))
            sender.setCurrentLimit(self.phidget_config.current_limit)
            sender.setVelocityLimit(self.phidget_config.velocity_limit)
            sender.setAcceleration(self.phidget_config.acceleration)
            sender.setDeadBand(self.phidget_config.dead_band)
            sender.setKp(self.phidget_config.kp)
            sender.setKi(self.phidget_config.ki)
            sender.setKd(self.phidget_config.kd)
            sender.addPositionOffset(-sender.getPosition())
            sender.setEngaged(True)
            try:
                sender.enableFailsafe(self.phidget_config.failsafe_timeout)
            except Exception:
                pass
            with self._motor_lock:
                self._motor_ok = True
                self._phidgets_runaway_count = 0
            self.get_logger().info('MotorPositionController attached and engaged')

            if self.state == State.CONNECTING:
                self._transition_state(State.CALIBRATING)
        except PhidgetException as e:
            self.get_logger().error(f'MotorPositionController config failed: {e}')

    def _on_detach(self, sender):
        with self._motor_lock:
            self._motor_ok = False
            self._phidgets_controller = None
        try:
            sender.close()
        except Exception:
            pass
        self.get_logger().warn('MotorPositionController detached')
        if self.state == State.ARMED:
            self._transition_state(State.ERROR)

    def _command_position(self, normalized: float):
        if self.state != State.ARMED:
            return

        target_degrees = normalized * self.phidget_config.motor_degrees_at_max_steer
        self.current_target = target_degrees

    def _apply_ramp(self, dt: float):
        error = self.current_target - self.smoothed_target

        if abs(error) < 0.5:
            self.smoothed_target = self.current_target
            return

        alpha = self.safety_config.smoothing_alpha
        sharpness = self.safety_config.scurve_sharpness

        max_range = self.phidget_config.motor_degrees_at_max_steer
        normalized_error = min(abs(error) / max_range, 1.0)
        scurve_factor = normalized_error ** sharpness
        dynamic_alpha = alpha + (1.0 - alpha) * scurve_factor

        max_step = self.safety_config.ramp_rate * max_range * dt
        weighted_step = error * dynamic_alpha

        if abs(weighted_step) > max_step:
            weighted_step = max_step if error > 0 else -max_step

        self.smoothed_target += weighted_step

    def _send_to_phidget(self):
        if self.state != State.ARMED:
            return

        with self._motor_lock:
            if not self._motor_ok or self._phidgets_controller is None:
                return
            ctrl = self._phidgets_controller

        try:
            ctrl.setTargetPosition(self.smoothed_target)
            ctrl.resetFailsafe()

            duty = abs(ctrl.getDutyCycle())
            pos_error = abs(ctrl.getPosition() - self.smoothed_target)

            if duty > self.safety_config.runaway_duty_threshold and pos_error > self.safety_config.runaway_position_error:
                self._phidgets_runaway_count += 1
                if self._phidgets_runaway_count > self.safety_config.runaway_max_count:
                    self.get_logger().error('Steering motor runaway detected — disengaging')
                    ctrl.setEngaged(False)
                    with self._motor_lock:
                        self._motor_ok = False
                    self._transition_state(State.ERROR)
            else:
                self._phidgets_runaway_count = 0
        except PhidgetException as e:
            self.get_logger().error(f'Phidget command failed: {e}')
            self._transition_state(State.ERROR)

    def _get_feedback(self) -> dict:
        with self._motor_lock:
            if not self._motor_ok or self._phidgets_controller is None:
                return {'pos': 0.0, 'vel': 0.0, 'duty': 0.0}
            ctrl = self._phidgets_controller

        try:
            pos = ctrl.getPosition()
            vel = ctrl.getVelocity()
            duty = ctrl.getDutyCycle()
            return {'pos': pos, 'vel': vel, 'duty': duty}
        except PhidgetException as e:
            self.get_logger().warn(f'Feedback read failed: {e}')
            return {'pos': 0.0, 'vel': 0.0, 'duty': 0.0}

    def _check_phidget_health(self) -> bool:
        with self._motor_lock:
            if not self._motor_ok or self._phidgets_controller is None:
                return False
            ctrl = self._phidgets_controller

        try:
            if not ctrl.getEngaged():
                self.get_logger().warn('Phidget motor not engaged')
                return False
            return True
        except PhidgetException:
            return False

    def _emergency_stop(self):
        self.get_logger().warn('Emergency stop activated')
        self._transition_state(State.ESTOP)

        with self._motor_lock:
            if self._motor_ok and self._phidgets_controller is not None:
                try:
                    self._phidgets_controller.setTargetPosition(0.0)
                    self._phidgets_controller.setEngaged(False)
                    self._motor_ok = False
                except PhidgetException as e:
                    self.get_logger().error(f'E-stop command failed: {e}')

        self._write_serial('Z4')

    def _clear_estop(self) -> bool:
        if self.state != State.ESTOP:
            return False

        self.get_logger().info('Clearing emergency stop')

        with self._motor_lock:
            if self._phidgets_controller is not None:
                try:
                    self._phidgets_controller.setEngaged(True)
                    try:
                        self._phidgets_controller.enableFailsafe(self.phidget_config.failsafe_timeout)
                    except Exception:
                        pass
                    self._motor_ok = True
                    self._phidgets_runaway_count = 0
                except PhidgetException as e:
                    self.get_logger().error(f'Failed to re-engage motor: {e}')
                    self._transition_state(State.ERROR)
                    return False
            else:
                self._transition_state(State.ERROR)
                return False

        self.current_target = 0.0
        self.smoothed_target = 0.0
        self._transition_state(State.ARMED)
        self._write_serial('Z3')
        return True

    def _transition_state(self, new_state: State):
        if self.state != new_state:
            self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
            self.state = new_state
            self._write_serial(f'Z{int(new_state)}')

            if new_state == State.ARMED:
                self.last_command_time = time.monotonic()
                self.armed_time = time.monotonic()

            msg = String()
            msg.data = new_state.name
            self.state_pub.publish(msg)

    def parse_command(self, msg: str) -> Optional[dict]:
        if not msg:
            return None

        cmd_type = msg[0]
        payload = msg[1:] if len(msg) > 1 else ""

        if cmd_type == 'S':
            try:
                value = int(payload)
                return {'type': 'steering', 'value': value}
            except ValueError:
                return None
        elif cmd_type == 'H':
            return {'type': 'heartbeat'}
        elif cmd_type == 'L':
            return {'type': 'rc_lost'}
        elif cmd_type == 'R':
            return {'type': 'rc_recovered'}
        elif cmd_type == 'E':
            return {'type': 'estop'}
        return None

    def _control_loop(self):
        if self.state == State.CONNECTING:
            with self._motor_lock:
                motor_ok = self._motor_ok
            if not motor_ok and self._phidgets_controller is None:
                self._init_phidgets()
            elif motor_ok:
                self._transition_state(State.CALIBRATING)
            return

        if self.state == State.CALIBRATING:
            with self._motor_lock:
                motor_ok = self._motor_ok
            if motor_ok:
                self.current_target = 0.0
                self.smoothed_target = 0.0
                self._transition_state(State.ARMED)
                self._write_serial('A')
            return

        if self.state not in [State.ARMED, State.ESTOP]:
            return

        while True:
            line = self._read_serial()
            if line is None:
                break

            cmd = self.parse_command(line)
            if cmd is None:
                continue

            self.last_command_time = time.monotonic()

            if cmd['type'] == 'steering':
                if self.state == State.ARMED and not self.autonomous_mode:
                    normalized = cmd['value'] / 1000.0
                    normalized = max(-1.0, min(1.0, normalized))
                    self._command_position(normalized)
                self._write_serial('A')

            elif cmd['type'] == 'heartbeat':
                self.last_command_time = time.monotonic()
                self._write_serial('A')

            elif cmd['type'] == 'rc_lost':
                if not self.autonomous_mode:
                    self.rc_lost = True
                    self.get_logger().warn('RC signal lost')
                    self._command_position(0.0)

            elif cmd['type'] == 'rc_recovered':
                self.rc_lost = False
                self.get_logger().info('RC signal recovered')

            elif cmd['type'] == 'estop':
                if not self.autonomous_mode:
                    self._emergency_stop()

        if self.state == State.ARMED:
            dt = 1.0 / self.safety_config.control_rate
            self._apply_ramp(dt)
            self._send_to_phidget()

            pos_msg = Float64()
            pos_msg.data = self.smoothed_target
            self.steering_pub.publish(pos_msg)

    def _feedback_loop(self):
        if self.state not in [State.ARMED, State.ESTOP]:
            return

        feedback = self._get_feedback()
        pos_deg = feedback['pos']
        vel_deg = feedback['vel']
        duty = feedback['duty']

        msg = f"F{pos_deg:.1f},{vel_deg:.1f},{duty:.3f}"
        self._write_serial(msg)
        self.last_feedback_time = time.monotonic()

    def _watchdog_callback(self):
        now = time.monotonic()

        if self.state == State.ARMED:
            time_since_armed = now - self.armed_time
            in_grace_period = time_since_armed < self.startup_grace_period

            time_since_cmd = now - self.last_command_time
            if time_since_cmd > self.safety_config.command_timeout and not in_grace_period:
                if not self._timeout_warned:
                    self.get_logger().warn(f'Command timeout ({time_since_cmd:.2f}s) - entering safe state')
                    self._timeout_warned = True
                self._command_position(0.0)
            else:
                self._timeout_warned = False

            if not self._check_phidget_health():
                self._transition_state(State.ERROR)

        if self.state == State.ERROR:
            if now - self.last_command_time > 0.5:
                self.get_logger().info('Attempting error recovery...')
                with self._motor_lock:
                    ctrl = self._phidgets_controller
                    if ctrl is not None:
                        try:
                            ctrl.addPositionOffset(-ctrl.getPosition())
                            ctrl.setEngaged(True)
                            try:
                                ctrl.enableFailsafe(self.phidget_config.failsafe_timeout)
                            except Exception:
                                pass
                            self._motor_ok = True
                            self._phidgets_runaway_count = 0
                            self.current_target = 0.0
                            self.smoothed_target = 0.0
                        except PhidgetException as e:
                            self.get_logger().warn(f'Recovery failed: {e}')
                            self._motor_ok = False

                with self._motor_lock:
                    motor_ok = self._motor_ok

                if motor_ok:
                    self._transition_state(State.ARMED)
                    self._write_serial('Z3')
                elif self._phidgets_controller is None:
                    self._init_phidgets()

    def _estop_callback(self, _request, response):
        self._emergency_stop()
        response.success = True
        response.message = 'Emergency stop activated'
        return response

    def _clear_estop_callback(self, _request, response):
        success = self._clear_estop()
        response.success = success
        response.message = 'E-stop cleared' if success else 'Failed to clear e-stop'
        return response

    def _arm_callback(self, _request, response):
        if self.state == State.ESTOP:
            success = self._clear_estop()
        elif self.state == State.ARMED:
            success = True
        else:
            success = False

        response.success = success
        response.message = 'Armed' if success else 'Failed to arm'
        return response

    def _cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = time.monotonic()

    def _process_gear_shift(self):
        if self._gear_shift_state == self.GEAR_SHIFT_IDLE:
            return

        now = time.monotonic()
        elapsed = now - self._gear_shift_time

        if self._gear_shift_state == self.GEAR_SHIFT_BRAKING:
            if elapsed >= 0.3:
                needs_neutral = (
                    (self.current_gear == 'reverse' and self._gear_shift_target == 'high') or
                    (self.current_gear == 'high' and self._gear_shift_target == 'reverse')
                )
                if needs_neutral:
                    self._write_serial('K1')
                    self._gear_shift_state = self.GEAR_SHIFT_TO_NEUTRAL
                    self._gear_shift_time = now
                else:
                    gear_code = {'reverse': '0', 'neutral': '1', 'high': '2'}[self._gear_shift_target]
                    self._write_serial(f'K{gear_code}')
                    self.current_gear = self._gear_shift_target
                    self._gear_shift_state = self.GEAR_SHIFT_IDLE

        elif self._gear_shift_state == self.GEAR_SHIFT_TO_NEUTRAL:
            if elapsed >= 0.3:
                gear_code = {'reverse': '0', 'neutral': '1', 'high': '2'}[self._gear_shift_target]
                self._write_serial(f'K{gear_code}')
                self.current_gear = self._gear_shift_target
                self._gear_shift_state = self.GEAR_SHIFT_IDLE

    def _request_gear_shift(self, target_gear: str):
        if target_gear == self.current_gear:
            return
        if self._gear_shift_state != self.GEAR_SHIFT_IDLE:
            return

        self._gear_shift_target = target_gear
        self._write_serial('J0,80')
        self._gear_shift_state = self.GEAR_SHIFT_BRAKING
        self._gear_shift_time = time.monotonic()

    def _autonomous_control_loop(self):
        if self.state != State.ARMED or not self.autonomous_mode:
            return

        if not self.autonomous_started:
            self._write_serial('AUTO')
            self._write_serial('K1')
            self.autonomous_started = True
            self.get_logger().info('Autonomous mode active, sent AUTO to Arduino')

        self._process_gear_shift()

        if self._gear_shift_state != self.GEAR_SHIFT_IDLE:
            return

        now = time.monotonic()
        if now - self.last_cmd_vel_time > 0.5:
            self._write_serial('J0,180')
            return

        linear_x = self.last_cmd_vel.linear.x
        angular_z = self.last_cmd_vel.angular.z

        if abs(linear_x) > 0.01 and abs(angular_z) > 0.001:
            steering_angle = math.atan(self.WHEELBASE * angular_z / linear_x)
            steering_angle = max(-self.MAX_STEERING_ANGLE, min(self.MAX_STEERING_ANGLE, steering_angle))
            normalized = steering_angle / self.MAX_STEERING_ANGLE
        else:
            normalized = 0.0
        self._command_position(normalized)

        if linear_x > 0.05:
            throttle = int(linear_x / self.MAX_SPEED * self.HW_THROTTLE_MAX)
            throttle = max(0, min(self.HW_THROTTLE_MAX, throttle))
            brake = self.HW_BRAKE_RELEASED
        elif linear_x < -0.05:
            throttle = int(abs(linear_x) / self.MAX_SPEED * self.HW_THROTTLE_MAX)
            throttle = max(0, min(self.HW_THROTTLE_MAX, throttle))
            brake = self.HW_BRAKE_RELEASED
        else:
            throttle = 0
            brake = self.HW_BRAKE_RELEASED
        self._write_serial(f'J{throttle},{brake}')

        if linear_x > 0.05:
            self._request_gear_shift('high')
        elif linear_x < -0.05:
            self._request_gear_shift('reverse')
        elif abs(linear_x) < 0.01:
            self._request_gear_shift('neutral')

    def _set_mode_callback(self, _request, response):
        self.autonomous_mode = not self.autonomous_mode
        if self.autonomous_mode:
            if not hasattr(self, 'cmd_vel_sub'):
                self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
                self.auto_timer = self.create_timer(1.0 / 50.0, self._autonomous_control_loop)
            self._write_serial('AUTO')
            self.autonomous_started = True
            self.current_gear = 'neutral'
            self._gear_shift_state = self.GEAR_SHIFT_IDLE
            response.message = 'Switched to autonomous mode'
        else:
            self._write_serial('MANUAL')
            self._command_position(0.0)
            self.autonomous_started = False
            self.current_gear = 'neutral'
            self._gear_shift_state = self.GEAR_SHIFT_IDLE
            response.message = 'Switched to manual mode'
        response.success = True
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self.get_logger().info('Shutting down...')

        if self.autonomous_mode:
            self._write_serial('MANUAL')

        if self._tak_socket is not None:
            try:
                self._tak_socket.close()
            except Exception:
                pass
            self._tak_socket = None

        with self._motor_lock:
            ctrl = self._phidgets_controller
            if ctrl is not None:
                try:
                    if self._motor_ok:
                        ctrl.setTargetPosition(0.0)
                        ctrl.setEngaged(False)
                    self._motor_ok = False
                    self._phidgets_controller = None
                    ctrl.close()
                except Exception:
                    pass

        if self.serial_conn is not None:
            try:
                self._write_serial('Z0')
                self.serial_conn.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RCPhidgetDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
