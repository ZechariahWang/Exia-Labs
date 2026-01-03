#!/usr/bin/env python3
# Xbox Teleop Node - Three-motor direct control (throttle, brake, steering)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Float64
from std_srvs.srv import Trigger


class ThreeMotorTeleopNode(Node):

    # Xbox controller axis mapping
    AXIS_LEFT_STICK_X = 0   # Steering
    AXIS_LEFT_STICK_Y = 1   # Unused
    AXIS_LT = 2             # Brake (1.0 = not pressed, -1.0 = fully pressed)
    AXIS_RIGHT_STICK_X = 3
    AXIS_RT = 5             # Throttle (1.0 = not pressed, -1.0 = fully pressed)
    AXIS_DPAD_Y = 7         # D-pad vertical (up=1, down=-1)

    # Button mapping
    BUTTON_A = 0      # Toggle enable
    BUTTON_B = 1      # E-stop
    BUTTON_X = 2      # Slow mode
    BUTTON_Y = 3      # Fast mode
    BUTTON_LB = 4     # Normal mode
    BUTTON_BACK = 6   # Reset steering trim
    BUTTON_START = 7  # Clear E-stop

    def __init__(self):
        super().__init__('xbox_teleop_three_motor')

        # Parameters
        self.declare_parameter('wheel_radius', 0.3)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('throttle_ramp_rate', 2.0)
        self.declare_parameter('brake_ramp_rate', 3.0)
        self.declare_parameter('steering_ramp_rate', 2.0)
        self.declare_parameter('slow_factor', 0.5)
        self.declare_parameter('normal_factor', 0.75)
        self.declare_parameter('fast_factor', 1.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.throttle_ramp_rate = self.get_parameter('throttle_ramp_rate').value
        self.brake_ramp_rate = self.get_parameter('brake_ramp_rate').value
        self.steering_ramp_rate = self.get_parameter('steering_ramp_rate').value
        self.slow_factor = self.get_parameter('slow_factor').value
        self.normal_factor = self.get_parameter('normal_factor').value
        self.fast_factor = self.get_parameter('fast_factor').value

        # State
        self.enabled = True
        self.estop_active = False
        self.speed_mode = self.normal_factor
        self.steering_trim = 0.0
        self.reverse_mode = False
        self._current_steering = 0.0
        self._current_throttle = 0.0
        self._current_brake = 0.0
        self._last_time = None
        self._prev_buttons = []
        self._prev_dpad_y = 0.0
        self._last_warn_time = 0.0

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
        self._last_joy_time = self.get_clock().now()

        self.get_logger().info('Xbox Teleop started - LT: brake, RT: throttle, Left stick: steering, D-pad down: reverse')

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    def _ramp_toward(self, current: float, target: float, rate: float, dt: float) -> float:
        max_change = rate * dt
        diff = target - current
        if abs(diff) <= max_change:
            return target
        return current + max_change if diff > 0 else current - max_change

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
        self._process_dpad(msg.axes)
        self._prev_buttons = list(msg.buttons)

        if not self.enabled or self.estop_active:
            self._publish_stop()
            return

        if len(msg.axes) < 6:
            return

        # Read controls
        # Left stick X for steering (positive = right, so negate for standard convention)
        left_x = self._apply_deadzone(msg.axes[self.AXIS_LEFT_STICK_X])

        # Triggers: 1.0 = not pressed, -1.0 = fully pressed
        # Convert to 0.0 = not pressed, 1.0 = fully pressed
        rt_raw = msg.axes[self.AXIS_RT]
        lt_raw = msg.axes[self.AXIS_LT]
        target_throttle = self._clamp((1.0 - rt_raw) / 2.0, 0.0, 1.0) * self.speed_mode
        target_brake = self._clamp((1.0 - lt_raw) / 2.0, 0.0, 1.0)

        # Apply ramping
        self._current_throttle = self._ramp_toward(self._current_throttle, target_throttle, self.throttle_ramp_rate, dt)
        self._current_brake = self._ramp_toward(self._current_brake, target_brake, self.brake_ramp_rate, dt)

        # Steering from left stick X
        target_steering = left_x * self.max_steering_angle + self.steering_trim
        self._current_steering = self._ramp_toward(self._current_steering, target_steering, self.steering_ramp_rate, dt)
        self._current_steering = self._clamp(self._current_steering, -self.max_steering_angle, self.max_steering_angle)

        # Send commands
        self._send_steering(self._current_steering)
        self._send_throttle(self._current_throttle)
        self._send_brake(self._current_brake)
        self._publish_debug()

    def _process_buttons(self, buttons: list):
        if self._button_pressed(buttons, self.BUTTON_A):
            self.enabled = not self.enabled
            self.get_logger().info(f'Teleop {"ENABLED" if self.enabled else "DISABLED"}')

        if self._button_pressed(buttons, self.BUTTON_B):
            self.get_logger().warn('EMERGENCY STOP!')
            self.estop_active = True
            self.enabled = False
            self._emergency_stop()

        if self._button_pressed(buttons, self.BUTTON_X):
            self.speed_mode = self.slow_factor
            self.get_logger().info(f'Slow mode ({int(self.slow_factor * 100)}%)')

        if self._button_pressed(buttons, self.BUTTON_Y):
            self.speed_mode = self.fast_factor
            self.get_logger().info(f'Fast mode ({int(self.fast_factor * 100)}%)')

        if self._button_pressed(buttons, self.BUTTON_LB):
            self.speed_mode = self.normal_factor
            self.get_logger().info(f'Normal mode ({int(self.normal_factor * 100)}%)')

        if self._button_pressed(buttons, self.BUTTON_START):
            if self.estop_active:
                self.estop_active = False
                self.enabled = True
                self.get_logger().info('E-stop cleared')
                if self.clear_estop_client.service_is_ready():
                    self.clear_estop_client.call_async(Trigger.Request())

        if self._button_pressed(buttons, self.BUTTON_BACK):
            self.steering_trim = 0.0
            self.get_logger().info('Steering trim reset')

    def _process_dpad(self, axes: list):
        if len(axes) <= self.AXIS_DPAD_Y:
            return
        dpad_y = axes[self.AXIS_DPAD_Y]
        # Detect D-pad down press (transition from 0 to -1)
        if dpad_y < -0.5 and self._prev_dpad_y > -0.5:
            self.reverse_mode = not self.reverse_mode
            self.get_logger().info(f'Reverse mode {"ON" if self.reverse_mode else "OFF"}')
        # Detect D-pad up press (transition from 0 to 1) - also turns off reverse
        elif dpad_y > 0.5 and self._prev_dpad_y < 0.5:
            if self.reverse_mode:
                self.reverse_mode = False
                self.get_logger().info('Reverse mode OFF')
        self._prev_dpad_y = dpad_y

    def _send_steering(self, angle: float):
        msg = Float64MultiArray()
        msg.data = [angle, angle]
        self.steering_pub.publish(msg)

    def _send_throttle(self, throttle_pct: float):
        wheel_vel = (throttle_pct * self.max_speed) / self.wheel_radius
        if self.reverse_mode:
            wheel_vel = -wheel_vel
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

    def _emergency_stop(self):
        throttle_msg = Float64MultiArray()
        throttle_msg.data = [0.0, 0.0]
        self.throttle_pub.publish(throttle_msg)

        brake_msg = Float64MultiArray()
        brake_msg.data = [1.0]
        self.brake_pub.publish(brake_msg)

        self._current_throttle = 0.0
        self._current_brake = 1.0

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
        dt = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if dt > 0.5 and self.enabled and not self.estop_active:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_warn_time > 5.0:
                self.get_logger().warn('No joystick input - stopping')
                self._last_warn_time = now
            self._publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = ThreeMotorTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
