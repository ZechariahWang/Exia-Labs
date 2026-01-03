#!/usr/bin/env python3
# PS4 DualShock 4 Teleop Node - Three-motor direct control (throttle, brake, steering)
# Sends servo commands to Arduino Mega via serial on /dev/ttyACM0

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Float64
from std_srvs.srv import Trigger
import serial
import termios
import time


# Exception types for serial communication errors
SERIAL_ERRORS = (serial.SerialException, OSError, termios.error)


class PS4TeleopNode(Node):

    # PS4 DualShock 4 controller axis mapping (xpadneo driver on Jetson)
    # Note: xpadneo maps PS4 controller with Xbox-style trigger values
    AXIS_LEFT_STICK_X = 0   # Steering (-1.0 left, +1.0 right)
    AXIS_LEFT_STICK_Y = 1   # Unused
    AXIS_L2 = 3             # Brake trigger (+1.0 released, -1.0 pressed) - Xbox-style
    AXIS_R2 = 4             # Throttle trigger (+1.0 released, -1.0 pressed) - Xbox-style

    # Button mapping (PS4 DualShock 4)
    BUTTON_CROSS = 0      # Toggle enable (X button)
    BUTTON_CIRCLE = 1     # E-stop (O button)
    BUTTON_SHARE = 8      # Reset steering trim
    BUTTON_OPTIONS = 9    # Clear E-stop

    def __init__(self):
        super().__init__('ps4_teleop_three_motor')

        # Parameters
        self.declare_parameter('wheel_radius', 0.3)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('throttle_ramp_rate', 5.0)  # Configurable ramp rate (units per second)
        self.declare_parameter('steering_ramp_rate', 10.0)  # Fast response
        self.declare_parameter('brake_ramp_rate', 8.0)  # Faster for safety

        # Soft-start / warmup parameters
        self.declare_parameter('max_initial_throttle', 0.3)  # 30% limit during warmup
        self.declare_parameter('warmup_duration', 3.0)  # Seconds before full throttle allowed

        # Serial/Arduino parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('steering_servo_min', 0)    # Servo angle for full left
        self.declare_parameter('steering_servo_max', 180)  # Servo angle for full right
        self.declare_parameter('steering_servo_center', 90)  # Servo angle for center
        self.declare_parameter('throttle_servo_min', 90)    # ESC neutral (no throttle)
        self.declare_parameter('throttle_servo_max', 180)  # ESC full throttle
        self.declare_parameter('brake_servo_min', 0)       # Servo angle for no brake
        self.declare_parameter('brake_servo_max', 180)     # Servo angle for full brake

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.throttle_ramp_rate = self.get_parameter('throttle_ramp_rate').value
        self.steering_ramp_rate = self.get_parameter('steering_ramp_rate').value
        self.brake_ramp_rate = self.get_parameter('brake_ramp_rate').value

        # Soft-start / warmup parameters
        self.max_initial_throttle = self.get_parameter('max_initial_throttle').value
        self.warmup_duration = self.get_parameter('warmup_duration').value

        # Serial/Arduino parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.steering_servo_min = self.get_parameter('steering_servo_min').value
        self.steering_servo_max = self.get_parameter('steering_servo_max').value
        self.steering_servo_center = self.get_parameter('steering_servo_center').value
        self.throttle_servo_min = self.get_parameter('throttle_servo_min').value
        self.throttle_servo_max = self.get_parameter('throttle_servo_max').value
        self.brake_servo_min = self.get_parameter('brake_servo_min').value
        self.brake_servo_max = self.get_parameter('brake_servo_max').value

        # State
        self.enabled = True
        self.estop_active = False
        self.steering_trim = 0.0
        self._current_steering = 0.0
        self._current_throttle = 0.0
        self._current_brake = 0.0
        self._last_time = None
        self._prev_buttons = []
        self._last_warn_time = 0.0
        self._last_serial_time = 0.0
        self._serial_period = 0.02  # Send serial commands at 50Hz for smoother control
        self._last_arm_time = 0.0
        self._arm_period = 0.3  # Re-send ARM every 300ms to prevent timeout
        self._triggers_initialized = False
        self._startup_safety_passed = False

        # Warmup state for soft-start protection
        self._warmup_start_time = None  # When robot first started moving
        self._warmup_complete = False   # True after warmup_duration has passed

        # Hysteresis state for servo outputs (prevents 1-degree oscillation)
        self._last_steer_servo = 90
        self._last_throttle_servo = 90
        self._last_brake_servo = 0

        # Initialize serial connection to Arduino
        self.serial_conn = None
        self._init_serial()

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', qos)
        self.throttle_pub = self.create_publisher(Float64MultiArray, '/throttle_controller/commands', qos)
        self.brake_pub = self.create_publisher(Float64MultiArray, '/brake_controller/commands', qos)
        self.steering_debug_pub = self.create_publisher(Float64, '/teleop/steering_angle', 10)
        self.throttle_debug_pub = self.create_publisher(Float64, '/teleop/throttle', 10)
        self.brake_debug_pub = self.create_publisher(Float64, '/teleop/brake', 10)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)

        # Service clients
        self.estop_client = self.create_client(Trigger, '/ackermann/emergency_stop')
        self.clear_estop_client = self.create_client(Trigger, '/ackermann/clear_estop')

        # Safety timer
        self.create_timer(0.1, self._safety_timer_callback)
        # Keepalive timer - sends ARM and current position every 200ms to prevent Arduino timeout
        self.create_timer(0.2, self._keepalive_timer_callback)
        self._last_joy_time = self.get_clock().now()

        self.get_logger().info('PS4 Teleop started - R2: throttle, L2: brake, Left stick: steering')
        self.get_logger().info('Controls: X=enable, Circle=E-stop, Options=clear E-stop, Share=reset trim')
        self.get_logger().info(f'Serial: {self.serial_port} @ {self.baud_rate} baud')
        self.get_logger().info(
            f'Soft-start: {self.max_initial_throttle * 100:.0f}% max throttle for first {self.warmup_duration:.1f}s')

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _apply_trigger_deadzone(self, raw_value: float) -> float:
        """Convert trigger value from [+1.0, -1.0] to [0.0, 1.0] with deadzone.

        Triggers (via Wireless Controller driver): +1.0 = not pressed, -1.0 = fully pressed
        Output: 0.0 = not pressed, 1.0 = fully pressed
        """
        pressed = (1.0 - raw_value) / 2.0
        pressed = self._clamp(pressed, 0.0, 1.0)
        trigger_deadzone = 0.08  # 8% deadzone to filter controller noise
        if pressed < trigger_deadzone:
            return 0.0
        return (pressed - trigger_deadzone) / (1.0 - trigger_deadzone)

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    def _ramp_toward(self, current: float, target: float, rate: float, dt: float) -> float:
        max_change = rate * dt
        diff = target - current
        if abs(diff) <= max_change:
            return target
        return current + max_change if diff > 0 else current - max_change

    def _apply_warmup_limit(self, throttle: float) -> float:
        """Limit throttle during warmup period to protect hardware from sudden acceleration.

        During the first warmup_duration seconds of movement, throttle is limited
        to max_initial_throttle (default 30%). After warmup, full throttle is available.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # Start warmup timer when first non-zero throttle is detected
        if throttle > 0.05:
            if self._warmup_start_time is None:
                self._warmup_start_time = now
                self.get_logger().info(
                    f'Warmup started - throttle limited to {self.max_initial_throttle * 100:.0f}% '
                    f'for {self.warmup_duration:.1f}s')

        # Check if warmup is complete
        if self._warmup_start_time is not None:
            elapsed = now - self._warmup_start_time
            if elapsed >= self.warmup_duration:
                if not self._warmup_complete:
                    self._warmup_complete = True
                    self.get_logger().info('Warmup complete - full throttle available')
                return throttle  # No limit after warmup
            else:
                # During warmup, limit throttle
                return min(throttle, self.max_initial_throttle)

        return throttle

    def _reset_warmup(self):
        """Reset warmup state - called on E-stop, disable, or stop conditions."""
        if self._warmup_start_time is not None or self._warmup_complete:
            self._warmup_start_time = None
            self._warmup_complete = False
            self.get_logger().debug('Warmup state reset')

    def _init_serial(self):
        """Initialize serial connection to Arduino Mega."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            time.sleep(2.0)  # Wait for Arduino to reset after serial connection
            self.get_logger().info(f'Serial connected to {self.serial_port} at {self.baud_rate} baud')
            # Send ARM command to Arduino
            self._send_arm_command()
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.serial_conn = None

    def _send_arm_command(self):
        """Send ARM command to Arduino to enable servo control."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            self.serial_conn.write(b"ARM\n")
            self.serial_conn.flush()
            time.sleep(0.1)
            self.get_logger().info('Sent ARM command to Arduino')
        except SERIAL_ERRORS as e:
            self.get_logger().warn(f'Failed to send ARM: {e}')

    def _send_disarm_command(self):
        """Send DISARM command to Arduino."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            self.serial_conn.write(b"DISARM\n")
            self.serial_conn.flush()
            self.get_logger().info('Sent DISARM command to Arduino')
        except SERIAL_ERRORS as e:
            self.get_logger().warn(f'Failed to send DISARM: {e}')

    def _send_servo_commands(self, steering_angle: int, throttle_angle: int, brake_angle: int, force: bool = False):
        """Send servo angles to Arduino. Format: S<steer>,T<throttle>,B<brake>\n"""
        now = time.time()

        if self.serial_conn is None or not self.serial_conn.is_open:
            # Log disconnection warning once per second
            if (now - self._last_warn_time) > 1.0:
                self.get_logger().warn('Serial disconnected - commands not being sent')
                self._last_warn_time = now
            return

        # Periodically re-send ARM to prevent Arduino timeout
        if (now - self._last_arm_time) > self._arm_period:
            try:
                self.serial_conn.write(b"ARM\n")
                self._last_arm_time = now
            except SERIAL_ERRORS:
                pass  # Will be caught below

        # Rate limit serial commands (unless forced)
        if not force and (now - self._last_serial_time) < self._serial_period:
            return
        self._last_serial_time = now

        try:
            cmd = f"S{steering_angle},T{throttle_angle},B{brake_angle}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
            self.serial_conn.flush()
            # Debug: log every second what we're sending
            if (now - self._last_warn_time) > 1.0:
                self.get_logger().info(
                    f'Sent: S{steering_angle} T{throttle_angle} B{brake_angle} | '
                    f'throttle={self._current_throttle:.2f} brake={self._current_brake:.2f}')
                self._last_warn_time = now
        except SERIAL_ERRORS as e:
            self.get_logger().warn(f'Serial write failed: {e}')
            self.serial_conn = None  # Mark as disconnected

    def _steering_to_servo(self, steering_rad: float) -> int:
        """Convert steering angle (radians) to servo angle (0-180) with hysteresis."""
        # Map from [-max_steering, +max_steering] to [min, max]
        # Negative steering = left, positive = right
        normalized = (steering_rad + self.max_steering_angle) / (2.0 * self.max_steering_angle)
        normalized = self._clamp(normalized, 0.0, 1.0)
        servo_angle = self.steering_servo_min + normalized * (self.steering_servo_max - self.steering_servo_min)
        new_angle = int(round(servo_angle))
        # Hysteresis: only change if difference > 1 degree (prevents oscillation)
        if abs(new_angle - self._last_steer_servo) > 1:
            self._last_steer_servo = new_angle
        return self._last_steer_servo

    def _throttle_to_servo(self, throttle_pct: float) -> int:
        """Convert throttle percentage (0-1) to servo angle with hysteresis."""
        throttle_pct = self._clamp(throttle_pct, 0.0, 1.0)
        servo_angle = self.throttle_servo_min + throttle_pct * (self.throttle_servo_max - self.throttle_servo_min)
        new_angle = int(round(servo_angle))
        # Hysteresis: only change if difference > 1 degree
        if abs(new_angle - self._last_throttle_servo) > 1:
            self._last_throttle_servo = new_angle
        return self._last_throttle_servo

    def _brake_to_servo(self, brake_pct: float) -> int:
        """Convert brake percentage (0-1) to servo angle with hysteresis."""
        brake_pct = self._clamp(brake_pct, 0.0, 1.0)
        servo_angle = self.brake_servo_min + brake_pct * (self.brake_servo_max - self.brake_servo_min)
        new_angle = int(round(servo_angle))
        # Hysteresis: only change if difference > 1 degree
        if abs(new_angle - self._last_brake_servo) > 1:
            self._last_brake_servo = new_angle
        return self._last_brake_servo

    def _button_pressed(self, buttons: list, button_id: int) -> bool:
        if button_id >= len(buttons):
            return False
        current = buttons[button_id] == 1
        previous = self._prev_buttons[button_id] == 1 if button_id < len(self._prev_buttons) else False
        return current and not previous

    def _joy_callback(self, msg: Joy):
        self._last_joy_time = self.get_clock().now()

        # Calculate dt
        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.02 if self._last_time is None else now - self._last_time
        self._last_time = now
        dt = self._clamp(dt, 0.001, 0.1)

        self._process_buttons(msg.buttons)
        self._prev_buttons = list(msg.buttons)

        if not self.enabled or self.estop_active:
            self._publish_stop()
            return

        if len(msg.axes) < 5:
            return

        # Read controls - Left stick X for steering
        left_x = self._apply_deadzone(msg.axes[self.AXIS_LEFT_STICK_X])

        # Triggers (via xpadneo): +1.0 = not pressed, -1.0 = fully pressed (Xbox-style)
        r2_raw = msg.axes[self.AXIS_R2]
        l2_raw = msg.axes[self.AXIS_L2]

        # Trigger initialization: wait for non-zero value
        if not self._triggers_initialized:
            if r2_raw != 0.0 or l2_raw != 0.0:
                self._triggers_initialized = True
            else:
                # Until initialized, treat as not pressed (+1.0 = released for Xbox-style)
                r2_raw = 1.0
                l2_raw = 1.0

        # Convert triggers to 0.0-1.0 range with deadzone
        target_throttle = self._apply_trigger_deadzone(r2_raw)
        target_brake = self._apply_trigger_deadzone(l2_raw)

        # Startup safety: require triggers to be released before enabling control
        if not self._startup_safety_passed:
            if target_throttle < 0.1 and target_brake < 0.1:
                self._startup_safety_passed = True
                self.get_logger().info('Startup safety passed - controls enabled')
            else:
                self._publish_stop()
                return

        # Smooth ramping for throttle/brake to reduce jitter
        # Uses configurable ramp rates from ROS parameters
        self._current_throttle = self._ramp_toward(self._current_throttle, target_throttle, self.throttle_ramp_rate, dt)
        self._current_brake = self._ramp_toward(self._current_brake, target_brake, self.brake_ramp_rate, dt)

        # Apply warmup limit - limits throttle during initial movement period
        self._current_throttle = self._apply_warmup_limit(self._current_throttle)

        # Steering from left stick X - direct control
        target_steering = left_x * self.max_steering_angle + self.steering_trim
        self._current_steering = self._clamp(target_steering, -self.max_steering_angle, self.max_steering_angle)

        # Send commands
        self._send_steering(self._current_steering)
        self._send_throttle(self._current_throttle)
        self._send_brake(self._current_brake)
        self._publish_debug()

        # Send servo commands to Arduino
        steering_servo = self._steering_to_servo(self._current_steering)
        throttle_servo = self._throttle_to_servo(self._current_throttle)
        brake_servo = self._brake_to_servo(self._current_brake)
        self._send_servo_commands(steering_servo, throttle_servo, brake_servo)

    def _process_buttons(self, buttons: list):
        if self._button_pressed(buttons, self.BUTTON_CROSS):
            self.enabled = not self.enabled
            self.get_logger().info(f'Teleop {"ENABLED" if self.enabled else "DISABLED"}')
            if not self.enabled:
                # Reset warmup when disabled for safety on re-enable
                self._reset_warmup()

        if self._button_pressed(buttons, self.BUTTON_CIRCLE):
            self.get_logger().warn('EMERGENCY STOP!')
            self.estop_active = True
            self.enabled = False
            self._reset_warmup()  # Reset warmup on E-stop
            self._emergency_stop()

        if self._button_pressed(buttons, self.BUTTON_OPTIONS):
            if self.estop_active:
                self.estop_active = False
                self.enabled = True
                # Note: warmup already reset by E-stop, will start fresh
                self.get_logger().info('E-stop cleared - warmup will restart on throttle')
                if self.clear_estop_client.service_is_ready():
                    self.clear_estop_client.call_async(Trigger.Request())

        if self._button_pressed(buttons, self.BUTTON_SHARE):
            self.steering_trim = 0.0
            self.get_logger().info('Steering trim reset')

    def _send_steering(self, angle: float):
        msg = Float64MultiArray()
        msg.data = [angle, angle]
        self.steering_pub.publish(msg)

    def _send_throttle(self, throttle_pct: float):
        wheel_vel = (throttle_pct * self.max_speed) / self.wheel_radius
        msg = Float64MultiArray()
        msg.data = [wheel_vel, wheel_vel]
        self.throttle_pub.publish(msg)

    def _send_brake(self, brake_pct: float):
        msg = Float64MultiArray()
        msg.data = [self._clamp(brake_pct, 0.0, 1.0)]
        self.brake_pub.publish(msg)

    def _publish_stop(self):
        throttle_msg = Float64MultiArray()
        throttle_msg.data = [0.0, 0.0]
        self.throttle_pub.publish(throttle_msg)

        brake_msg = Float64MultiArray()
        brake_msg.data = [0.3]
        self.brake_pub.publish(brake_msg)

        steering_msg = Float64MultiArray()
        steering_msg.data = [self._current_steering, self._current_steering]
        self.steering_pub.publish(steering_msg)

        # Send servo stop commands to Arduino
        steering_servo = self._steering_to_servo(self._current_steering)
        throttle_servo = self._throttle_to_servo(0.0)
        brake_servo = self._brake_to_servo(0.3)
        self._send_servo_commands(steering_servo, throttle_servo, brake_servo)

    def _emergency_stop(self):
        throttle_msg = Float64MultiArray()
        throttle_msg.data = [0.0, 0.0]
        self.throttle_pub.publish(throttle_msg)

        brake_msg = Float64MultiArray()
        brake_msg.data = [1.0]
        self.brake_pub.publish(brake_msg)

        self._current_throttle = 0.0
        self._current_brake = 1.0

        # Send servo emergency stop commands to Arduino (full brake, no throttle)
        steering_servo = self._steering_to_servo(self._current_steering)
        throttle_servo = self._throttle_to_servo(0.0)
        brake_servo = self._brake_to_servo(1.0)
        self._send_servo_commands(steering_servo, throttle_servo, brake_servo, force=True)

        if self.estop_client.service_is_ready():
            self.estop_client.call_async(Trigger.Request())

    def _publish_debug(self):
        msg = Float64()
        msg.data = self._current_steering
        self.steering_debug_pub.publish(msg)

        msg = Float64()
        msg.data = self._current_throttle
        self.throttle_debug_pub.publish(msg)

        msg = Float64()
        msg.data = self._current_brake
        self.brake_debug_pub.publish(msg)

    def _safety_timer_callback(self):
        """Read and log Arduino responses for debugging."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        try:
            # Read any available responses from Arduino
            while self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Log important messages
                    if 'TIMEOUT' in line or 'IGNORED' in line or 'DISARM' in line:
                        self.get_logger().warn(f'Arduino: {line}')
                    elif 'ARMED' in line:
                        self.get_logger().info(f'Arduino: {line}')
        except SERIAL_ERRORS:
            pass  # Handled elsewhere

    def _keepalive_timer_callback(self):
        """Send keepalive to Arduino to prevent timeout, even when no joystick input."""
        if self.serial_conn is None:
            return

        try:
            if not self.serial_conn.is_open:
                self.get_logger().warn('Serial port closed unexpectedly')
                self.serial_conn = None
                return
        except SERIAL_ERRORS:
            self.serial_conn = None
            return

        if not self.enabled or self.estop_active:
            return

        try:
            # Send ARM to keep Arduino alive
            self.serial_conn.write(b"ARM\n")
            # Also send current servo positions
            steering_servo = self._steering_to_servo(self._current_steering)
            throttle_servo = self._throttle_to_servo(self._current_throttle)
            brake_servo = self._brake_to_servo(self._current_brake)
            cmd = f"S{steering_servo},T{throttle_servo},B{brake_servo}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
            self.serial_conn.flush()  # Force send immediately
        except SERIAL_ERRORS as e:
            self.get_logger().warn(f'Keepalive failed: {e}')
            self.serial_conn = None

    def close_serial(self):
        """Close the serial connection to Arduino."""
        if self.serial_conn is not None:
            try:
                if self.serial_conn.is_open:
                    self._send_disarm_command()
                    time.sleep(0.1)
                    self.serial_conn.close()
                    self.get_logger().info('Serial connection closed')
            except SERIAL_ERRORS:
                pass  # Already disconnected


def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
