#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
import serial
import time
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

@dataclass
class ODriveConfig:
    vel_limit: float = 30.0
    accel_limit: float = 1.0
    current_limit: float = 10.0
    position_gain: float = 20.0
    velocity_gain: float = 0.16
    velocity_integrator_gain: float = 0.32

@dataclass
class SafetyConfig:
    max_position: float = 6
    command_timeout: float = 0.5
    feedback_rate: float = 20.0
    control_rate: float = 50.0
    ramp_rate: float = 20.0

class RCDriverControlNode(Node):
    def __init__(self):
        super().__init__('rc_driver_control_node')

        self._declare_parameters()

        self.state = State.IDLE
        self.last_command_time = time.monotonic()
        self.last_feedback_time = time.monotonic()
        self.anchor_position = 0.0
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
        )

        self.steering_pub = self.create_publisher(Float64, '/odrive/steering_position', 10)
        self.state_pub = self.create_publisher(String, '/odrive/state', 10)

        self.estop_srv = self.create_service(Trigger, '/odrive/emergency_stop', self._estop_callback)
        self.clear_srv = self.create_service(Trigger, '/odrive/clear_estop', self._clear_estop_callback)
        self.arm_srv = self.create_service(Trigger, '/odrive/arm', self._arm_callback)

        control_period = 1.0 / self.safety_config.control_rate
        feedback_period = 1.0 / self.safety_config.feedback_rate
        self.control_timer = self.create_timer(control_period, self._control_loop)
        self.feedback_timer = self.create_timer(feedback_period, self._feedback_loop)
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self._init_serial()
        self._transition_state(State.CONNECTING)

        self.get_logger().info('RC Driver Control Node started')

    def _declare_parameters(self):
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('vel_limit', 30.0)
        self.declare_parameter('accel_limit', 1.0)
        self.declare_parameter('current_limit', 10.0)
        self.declare_parameter('position_gain', 20.0)
        self.declare_parameter('velocity_gain', 0.16)
        self.declare_parameter('velocity_integrator_gain', 0.32)
        self.declare_parameter('max_position', 6)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('feedback_rate', 20.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('ramp_rate', 20.0)
        self.declare_parameter('turns_per_steering_rad', 1.0)

    def _init_serial(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.01
            )
            self.get_logger().info(f'Serial connected: {self.serial_port}')
            self.get_logger().info('Waiting for Arduino to boot...')
            time.sleep(2.0)
            self.serial_conn.reset_input_buffer()
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
            ctrl = self.axis.controller
            ctrl.config.control_mode = ControlMode.POSITION_CONTROL
            ctrl.config.input_mode = InputMode.PASSTHROUGH
            ctrl.config.vel_limit = self.odrive_config.vel_limit
            self.get_logger().info(f'ODrive configured: vel_limit={self.odrive_config.vel_limit}')
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
            timeout = 5.0
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

    def _set_anchor(self):
        if self.axis is None:
            return

        try:
            self.anchor_position = self.axis.pos_estimate
            self.current_target = 0.0
            self.smoothed_target = 0.0
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
        max_step = self.safety_config.ramp_rate * dt
        error = self.current_target - self.smoothed_target

        if abs(error) <= max_step:
            self.smoothed_target = self.current_target
        else:
            self.smoothed_target += max_step if error > 0 else -max_step

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
                self.get_logger().error(f'ODrive errors: 0x{self.axis.active_errors:X}')
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

            msg = String()
            msg.data = new_state.name
            self.state_pub.publish(msg)

    def _parse_command(self, msg: str) -> Optional[dict]:
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

            cmd = self._parse_command(line)
            if cmd is None:
                continue

            self.last_command_time = time.monotonic()

            if cmd['type'] == 'steering':
                if self.state == State.ARMED:
                    normalized = cmd['value'] / 1000.0
                    normalized = max(-1.0, min(1.0, normalized))
                    self._command_position(normalized)
                    self._write_serial('A')

            elif cmd['type'] == 'heartbeat':
                self.last_command_time = time.monotonic()
                self._write_serial('A')

            elif cmd['type'] == 'rc_lost':
                self.rc_lost = True
                self.get_logger().warn('RC signal lost')
                self._command_position(0.0)

            elif cmd['type'] == 'rc_recovered':
                self.rc_lost = False
                self.get_logger().info('RC signal recovered')

            elif cmd['type'] == 'estop':
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
            time_since_cmd = now - self.last_command_time
            if time_since_cmd > self.safety_config.command_timeout:
                if not hasattr(self, '_timeout_warned') or not self._timeout_warned:
                    self.get_logger().warn(f'Command timeout ({time_since_cmd:.2f}s) - entering safe state')
                    self._timeout_warned = True
                self._command_position(0.0)
            else:
                self._timeout_warned = False

            if not self._check_odrive_errors():
                self._transition_state(State.ERROR)

        if self.state == State.ERROR:
            if now - self.last_command_time > 5.0:
                self.get_logger().info('Attempting error recovery...')
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

    def destroy_node(self):
        self.get_logger().info('Shutting down...')

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
