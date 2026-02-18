#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import serial
import time
import math
import threading
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional

try:
    import odrive
    from odrive.enums import AxisState, ControlMode, InputMode
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False

class State(IntEnum):
    IDLE = 0
    CONNECTING = 1
    CALIBRATING = 2
    ARMED = 3
    ESTOP = 4
    ERROR = 5

cur_lim = 20

@dataclass
class ODriveConfig:
    vel_limit: float = 15.0
    accel_limit: float = 1.0
    current_limit: float = cur_lim
    torque_limit: float = 0.01
    max_torque: float = 0.01
    position_gain: float = 20.0
    velocity_gain: float = 0.16
    velocity_integrator_gain: float = 0.32

@dataclass
class SafetyConfig:
    max_position: float = 6
    command_timeout: float = 1.0
    feedback_rate: float = 20.0
    control_rate: float = 50.0
    ramp_rate: float = 10.0
    smoothing_alpha: float = 0.1
    scurve_sharpness: float = 3.0

class RCDriverControlNode(Node):
    def __init__(self):

        super().__init__('rc_driver_control_node')

        self._declare_parameters()

        self.state = State.IDLE
        self.last_command_time = time.monotonic()
        self.last_feedback_time = time.monotonic()
        self.armed_time = 0.0
        self.startup_grace_period = 15.0
        self.anchor_position = 0.0
        self.anchor_initialized = False
        self.current_target = 0.0
        self.smoothed_target = 0.0
        self.rc_lost = False
        self.odrive = None
        self.axis = None

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.serial_conn = None
        self.serial_buffer = ""
        self.serial_lock = threading.Lock()

        self.odrive_config = ODriveConfig(
            vel_limit=self.get_parameter('vel_limit').value,
            accel_limit=self.get_parameter('accel_limit').value,
            current_limit=self.get_parameter('current_limit').value,
            torque_limit=self.get_parameter('torque_limit').value,
            max_torque=self.get_parameter('max_torque').value,
            position_gain=self.get_parameter('position_gain').value,
            velocity_gain=self.get_parameter('velocity_gain').value,
            velocity_integrator_gain=self.get_parameter('velocity_integrator_gain').value,
        )

        self.safety_config = SafetyConfig(
            max_position=self.get_parameter('max_position').value,
            command_timeout=self.get_parameter('command_timeout').value,
            feedback_rate=self.get_parameter('feedback_rate').value,
            control_rate=self.get_parameter('control_rate').value,
            ramp_rate=self.get_parameter('ramp_rate').value,
            smoothing_alpha=self.get_parameter('smoothing_alpha').value,
            scurve_sharpness=self.get_parameter('scurve_sharpness').value,
        )

        self.steering_pub = self.create_publisher(Float64, '/odrive/steering_position', 10)
        self.state_pub = self.create_publisher(String, '/odrive/state', 10)

        self.estop_srv = self.create_service(Trigger, '/odrive/emergency_stop', self._estop_callback)
        self.clear_srv = self.create_service(Trigger, '/odrive/clear_estop', self._clear_estop_callback)
        self.arm_srv = self.create_service(Trigger, '/odrive/arm', self._arm_callback)
        self.set_mode_srv = self.create_service(Trigger, '/driver/set_mode', self._set_mode_callback)

        self.autonomous_mode = self.get_parameter('autonomous_mode').value
        self.WHEELBASE = 1.3
        self.MAX_STEERING_ANGLE = 0.6
        self.MAX_SPEED = 5.0
        self.HW_THROTTLE_MAX = 75
        self.HW_BRAKE_FULL = 80
        self.HW_BRAKE_RELEASED = 180
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = time.monotonic()
        self.current_gear = 'neutral'
        self.autonomous_started = False

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
            self.get_logger().info('Sent AUTO to Arduino early (before ODrive connect)')
        self.autonomous_grace_time = time.monotonic()
        self._transition_state(State.CONNECTING)

        self.get_logger().info('RC Driver Control Node started')

    def _declare_parameters(self):
        self.declare_parameter('serial_port', '/dev/arduino_control')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('vel_limit', 15.0)
        self.declare_parameter('accel_limit', 1.0)
        self.declare_parameter('current_limit', cur_lim)
        self.declare_parameter('torque_limit', 0.01)
        self.declare_parameter('max_torque', 0.01)
        self.declare_parameter('position_gain', 20.0)
        self.declare_parameter('velocity_gain', 0.16)
        self.declare_parameter('velocity_integrator_gain', 0.32)
        self.declare_parameter('max_position', 6)
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('feedback_rate', 20.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('ramp_rate', 10.0)
        self.declare_parameter('smoothing_alpha', 0.1)
        self.declare_parameter('scurve_sharpness', 3.0)
        self.declare_parameter('turns_per_steering_rad', 1.0)
        self.declare_parameter('autonomous_mode', False)

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
            self.get_logger().info('Serial ready (no handshake)')
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

    def _connect_odrive(self) -> bool:
        if not ODRIVE_AVAILABLE:
            self.get_logger().error('ODrive library not available')
            return False

        try:
            self.get_logger().info('Searching for ODrive...')
            self.odrive = odrive.find_any(timeout=10)
            if self.odrive is None:
                self.get_logger().error('ODrive not found')
                return False

            self.axis = self.odrive.axis0
            self.get_logger().info('ODrive connected')

            self._configure_odrive()
            return True
        except Exception as e:
            self.get_logger().error(f'ODrive connection failed: {e}')
            return False

    def _configure_odrive(self):
        if self.axis is None:
            return

        try:
            self.axis.config.motor.current_soft_max = self.odrive_config.current_limit
            self.axis.config.motor.current_hard_max = self.odrive_config.current_limit + 10.0
            self.get_logger().info(f'Set current_soft_max={self.odrive_config.current_limit}A, current_hard_max={self.odrive_config.current_limit + 10.0}A')
        except Exception as e:
            self.get_logger().warn(f'Could not set current limits: {e}')

        try:
            ctrl = self.axis.controller
            ctrl.config.control_mode = ControlMode.POSITION_CONTROL
            ctrl.config.input_mode = InputMode.PASSTHROUGH
            ctrl.config.vel_limit = self.odrive_config.vel_limit
            self.get_logger().info(f'ODrive configured: POSITION_CONTROL, vel_limit={self.odrive_config.vel_limit}')
        except Exception as e:
            self.get_logger().warn(f'ODrive configuration warning: {e}')

    def _calibrate_odrive(self) -> bool:
        if self.axis is None:
            return False

        try:
            if self.axis.current_state == AxisState.CLOSED_LOOP_CONTROL:
                self.get_logger().info('ODrive already in closed-loop control')
                return True

            self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            timeout = 2
            start = time.monotonic()
            while self.axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
                if time.monotonic() - start > timeout:
                    self.get_logger().error('ODrive calibration timeout')
                    return False
                time.sleep(0.1)

            self.get_logger().info('ODrive in closed-loop control')
            return True
        except Exception as e:
            self.get_logger().error(f'ODrive calibration failed: {e}')
            return False

    def _set_anchor(self, force=False):
        if self.axis is None:
            return

        if self.anchor_initialized and not force:
            self.get_logger().info(f'Anchor already set at {self.anchor_position:.3f} rev, skipping')
            return

        try:
            self.anchor_position = self.axis.pos_estimate
            self.current_target = 0.0
            self.smoothed_target = 0.0
            self.anchor_initialized = True
            self.get_logger().info(f'Anchor set at {self.anchor_position:.3f} rev')
        except Exception as e:
            self.get_logger().error(f'Failed to set anchor: {e}')

    def _command_position(self, normalized: float):
        if self.axis is None or self.state != State.ARMED:
            return

        turns_per_rad = self.get_parameter('turns_per_steering_rad').value
        max_pos = self.safety_config.max_position

        target_rad = normalized * max_pos
        target_turns = target_rad * turns_per_rad

        self.current_target = target_turns

    def _apply_ramp(self, dt: float):
        error = self.current_target - self.smoothed_target

        if abs(error) < 0.001:
            self.smoothed_target = self.current_target
            return

        alpha = self.safety_config.smoothing_alpha
        sharpness = self.safety_config.scurve_sharpness

        normalized_error = min(abs(error) / self.safety_config.max_position, 1.0)
        scurve_factor = normalized_error ** sharpness
        dynamic_alpha = alpha + (1.0 - alpha) * scurve_factor

        max_step = self.safety_config.ramp_rate * dt
        weighted_step = error * dynamic_alpha

        if abs(weighted_step) > max_step:
            weighted_step = max_step if error > 0 else -max_step

        self.smoothed_target += weighted_step

    def _send_to_odrive(self):
        if self.axis is None or self.state != State.ARMED:
            return

        try:
            absolute_pos = self.anchor_position + self.smoothed_target
            self.axis.controller.input_pos = absolute_pos
        except Exception as e:
            self.get_logger().error(f'ODrive command failed: {e}')
            self._transition_state(State.ERROR)

    def _get_feedback(self) -> dict:
        if self.axis is None:
            return {'pos': 0.0, 'vel': 0.0, 'err': 1}

        try:
            pos = self.axis.pos_estimate - self.anchor_position
            vel = self.axis.vel_estimate
            err = self.axis.active_errors
            return {'pos': pos, 'vel': vel, 'err': err}
        except Exception as e:
            self.get_logger().warn(f'Feedback read failed: {e}')
            return {'pos': 0.0, 'vel': 0.0, 'err': 1}

    def _check_odrive_errors(self) -> bool:
        if self.axis is None:
            return False

        try:
            if self.axis.active_errors != 0:
                err = self.axis.active_errors
                self.get_logger().error(f'ODrive error: 0x{err:08X}')
                if err & 0x800:
                    self.get_logger().error('  -> SPINOUT_DETECTED (position error too large)')
                if err & 0x1000:
                    self.get_logger().error('  -> CURRENT_LIMIT_VIOLATION')
                if err & 0x20:
                    self.get_logger().error('  -> DRV_FAULT (driver chip fault)')
                if err & 0x40:
                    self.get_logger().error('  -> MISSING_INPUT')
                if err & 0x08:
                    self.get_logger().error('  -> MISSING_ESTIMATE')
                return False
            if not self.axis.is_armed:
                self.get_logger().warn('ODrive disarmed')
                return False
            return True
        except Exception:
            return False

    def _emergency_stop(self):
        self.get_logger().warn('Emergency stop activated')
        self._transition_state(State.ESTOP)

        if self.axis is not None:
            try:
                self.axis.controller.input_pos = self.axis.pos_estimate
                time.sleep(0.1)
                self.axis.requested_state = AxisState.IDLE
            except Exception as e:
                self.get_logger().error(f'E-stop command failed: {e}')

        self._write_serial('Z4')

    def _clear_estop(self) -> bool:
        if self.state != State.ESTOP:
            return False

        self.get_logger().info('Clearing emergency stop')

        if self.odrive is not None:
            try:
                self.odrive.clear_errors()
            except Exception:
                pass

        if self._calibrate_odrive():
            self._set_anchor()
            self._transition_state(State.ARMED)
            self._write_serial('Z3')
            return True

        self._transition_state(State.ERROR)
        return False

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
            if self._connect_odrive():
                self._transition_state(State.CALIBRATING)
            return

        if self.state == State.CALIBRATING:
            if self._calibrate_odrive():
                self._set_anchor()
                self._transition_state(State.ARMED)
                self._write_serial('A')
            else:
                self._transition_state(State.ERROR)
            return

        if self.state not in [State.ARMED, State.ESTOP]:
            return

        lines_read = 0
        while True:
            line = self._read_serial()
            if line is None:
                break
            lines_read += 1
            self.get_logger().info(f'Serial RX: {line}')

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
                if self.autonomous_mode:
                    pass
                else:
                    self._emergency_stop()

        if self.state == State.ARMED:
            dt = 1.0 / self.safety_config.control_rate
            self._apply_ramp(dt)
            self._send_to_odrive()

            pos_msg = Float64()
            pos_msg.data = self.smoothed_target
            self.steering_pub.publish(pos_msg)

    def _feedback_loop(self):
        if self.state not in [State.ARMED, State.ESTOP]:
            return

        feedback = self._get_feedback()
        pos_deg = feedback['pos'] * 360.0
        vel_deg = feedback['vel'] * 360.0
        err = feedback['err']

        msg = f"F{pos_deg:.1f},{vel_deg:.1f},{err}"
        self._write_serial(msg)
        self.last_feedback_time = time.monotonic()

    def _watchdog_callback(self):
        now = time.monotonic()

        if self.state == State.ARMED:
            time_since_armed = now - self.armed_time
            in_grace_period = time_since_armed < self.startup_grace_period

            time_since_cmd = now - self.last_command_time
            if time_since_cmd > self.safety_config.command_timeout and not in_grace_period:
                if not hasattr(self, '_timeout_warned') or not self._timeout_warned:
                    self.get_logger().warn(f'Command timeout ({time_since_cmd:.2f}s) - entering safe state')
                    self._timeout_warned = True
                self._command_position(0.0)
            else:
                self._timeout_warned = False

            if not self._check_odrive_errors():
                self._transition_state(State.ERROR)

        if self.state == State.ERROR:
            if now - self.last_command_time > 0.25:
                self.get_logger().info('Attempting error recovery...')
                try:
                    if self.odrive is not None:
                        self.odrive.clear_errors()
                    if self._calibrate_odrive():
                        self._set_anchor()
                        self._transition_state(State.ARMED)
                        self._write_serial('Z3')
                except Exception as e:
                    self.get_logger().warn(f'Quick recovery failed: {e}, trying full reconnect')
                    if self._connect_odrive() and self._calibrate_odrive():
                        self._set_anchor()
                        self._transition_state(State.ARMED)
                        self._write_serial('Z3')

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

    def _autonomous_control_loop(self):
        if self.state != State.ARMED or not self.autonomous_mode:
            return

        if not self.autonomous_started:
            self._write_serial('AUTO')
            self._write_serial('K1')
            self.autonomous_started = True
            self.get_logger().info('Autonomous mode active, sent AUTO to Arduino')

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

        if linear_x > 0.05 and self.current_gear != 'high':
            if self.current_gear == 'reverse':
                self._write_serial('J0,80')
                time.sleep(0.3)
                self._write_serial('K1')
                time.sleep(0.3)
            self._write_serial('K2')
            self.current_gear = 'high'
        elif linear_x < -0.05 and self.current_gear != 'reverse':
            self._write_serial('J0,80')
            time.sleep(0.3)
            if self.current_gear != 'neutral':
                self._write_serial('K1')
                time.sleep(0.3)
            self._write_serial('K0')
            self.current_gear = 'reverse'
        elif abs(linear_x) < 0.01 and self.current_gear != 'neutral':
            self._write_serial('K1')
            self.current_gear = 'neutral'

    def _set_mode_callback(self, _request, response):
        self.autonomous_mode = not self.autonomous_mode
        if self.autonomous_mode:
            if not hasattr(self, 'cmd_vel_sub'):
                self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
                self.auto_timer = self.create_timer(1.0 / 50.0, self._autonomous_control_loop)
            self._write_serial('AUTO')
            self.autonomous_started = True
            self.current_gear = 'neutral'
            response.message = 'Switched to autonomous mode'
        else:
            self._write_serial('MANUAL')
            self._command_position(0.0)
            self.autonomous_started = False
            self.current_gear = 'neutral'
            response.message = 'Switched to manual mode'
        response.success = True
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self.get_logger().info('Shutting down...')

        if self.autonomous_mode:
            self._write_serial('MANUAL')

        if self.axis is not None:
            try:
                self.axis.controller.input_pos = self.axis.pos_estimate
                time.sleep(0.2)
                self.axis.requested_state = AxisState.IDLE
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

    node = RCDriverControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
