#!/usr/bin/env python3
"""
Xbox Controller Teleop Node

Maps Xbox One controller to robot control:
  - Left Stick Y: Forward/Backward speed
  - Right Stick X: Steering
  - Right Trigger (RT): Boost speed
  - Left Trigger (LT): Brake
  - A Button: Start/Stop (toggle)
  - B Button: Emergency stop
  - X Button: Slow mode
  - Y Button: Fast mode

Usage:
    # Terminal 1: Start simulation
    ros2 launch exia_ground_description exia_ground_sim.launch.py

    # Terminal 2: Start joy driver
    ros2 run joy joy_node

    # Terminal 3: Start teleop
    ros2 run exia_ground_description xbox_teleop_node.py

Author: Zechariah Wang
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

"""
note, idrk if this will work until tested on hardware

# Terminal 1: Start simulation
ros2 launch exia_ground_description exia_ground_sim.launch.py

# Terminal 2: Start joy driver
ros2 run joy joy_node

# Terminal 3: Start Xbox teleop
ros2 run exia_ground_description xbox_teleop_node.py

"""


class XboxTeleopNode(Node):
    """Xbox controller teleop for Ackermann robot."""

    # Xbox One Controller Mapping (may vary slightly)
    AXIS_LEFT_STICK_X = 0      # Left stick horizontal
    AXIS_LEFT_STICK_Y = 1      # Left stick vertical
    AXIS_RIGHT_STICK_X = 3     # Right stick horizontal
    AXIS_RIGHT_STICK_Y = 4     # Right stick vertical
    AXIS_LT = 2                # Left trigger (1 to -1)
    AXIS_RT = 5                # Right trigger (1 to -1)

    BUTTON_A = 0
    BUTTON_B = 1
    BUTTON_X = 2
    BUTTON_Y = 3
    BUTTON_LB = 4
    BUTTON_RB = 5
    BUTTON_BACK = 6
    BUTTON_START = 7
    BUTTON_XBOX = 8
    BUTTON_L_STICK = 9
    BUTTON_R_STICK = 10

    def __init__(self):
        super().__init__('xbox_teleop')

        # Parameters
        self.declare_parameter('max_speed', 2.0)          # m/s
        self.declare_parameter('max_angular', 0.5)        # rad/s
        self.declare_parameter('slow_factor', 0.3)        # Slow mode multiplier
        self.declare_parameter('boost_factor', 1.5)       # Boost multiplier
        self.declare_parameter('deadzone', 0.1)           # Stick deadzone

        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular = self.get_parameter('max_angular').value
        self.slow_factor = self.get_parameter('slow_factor').value
        self.boost_factor = self.get_parameter('boost_factor').value
        self.deadzone = self.get_parameter('deadzone').value

        # State
        self.enabled = True
        self.speed_mode = 1.0  # 1.0 = normal, slow_factor = slow, boost_factor = fast
        self.estop_active = False

        # Previous button states (for edge detection)
        self._prev_buttons = []

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)

        # Service clients (optional - for e-stop)
        self.estop_client = self.create_client(Trigger, '/ackermann/emergency_stop')
        self.clear_estop_client = self.create_client(Trigger, '/ackermann/clear_estop')

        self.get_logger().info('Xbox Teleop Node started')
        self.get_logger().info('  A: Toggle enable/disable')
        self.get_logger().info('  B: Emergency stop')
        self.get_logger().info('  X: Slow mode')
        self.get_logger().info('  Y: Fast mode')
        self.get_logger().info('  Left Stick: Forward/Back')
        self.get_logger().info('  Right Stick: Steering')
        self.get_logger().info('  RT: Boost | LT: Brake')

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _button_pressed(self, buttons: list, button_id: int) -> bool:
        """Check if button was just pressed (edge detection)."""
        if button_id >= len(buttons):
            return False
        current = buttons[button_id] == 1
        previous = self._prev_buttons[button_id] == 1 if button_id < len(self._prev_buttons) else False
        return current and not previous

    def _joy_callback(self, msg: Joy):
        """Process joystick input."""

        # Edge detection for buttons
        if self._button_pressed(msg.buttons, self.BUTTON_A):
            self.enabled = not self.enabled
            state = "ENABLED" if self.enabled else "DISABLED"
            self.get_logger().info(f'Teleop {state}')

        if self._button_pressed(msg.buttons, self.BUTTON_B):
            self.get_logger().warn('EMERGENCY STOP!')
            self.estop_active = True
            self.enabled = False
            self._publish_stop()
            # Call e-stop service if available
            if self.estop_client.service_is_ready():
                self.estop_client.call_async(Trigger.Request())

        if self._button_pressed(msg.buttons, self.BUTTON_X):
            self.speed_mode = self.slow_factor
            self.get_logger().info('Slow mode')

        if self._button_pressed(msg.buttons, self.BUTTON_Y):
            self.speed_mode = self.boost_factor
            self.get_logger().info('Fast mode')

        if self._button_pressed(msg.buttons, self.BUTTON_START):
            # Clear e-stop
            if self.estop_active:
                self.estop_active = False
                self.enabled = True
                self.get_logger().info('E-stop cleared')
                if self.clear_estop_client.service_is_ready():
                    self.clear_estop_client.call_async(Trigger.Request())

        if self._button_pressed(msg.buttons, self.BUTTON_LB):
            self.speed_mode = 1.0
            self.get_logger().info('Normal mode')

        # Store button state for next iteration
        self._prev_buttons = list(msg.buttons)

        # If disabled or e-stopped, publish zero velocity
        if not self.enabled or self.estop_active:
            self._publish_stop()
            return

        # Get axis values
        axes = msg.axes
        if len(axes) < 6:
            return

        # Apply deadzone
        forward = self._apply_deadzone(axes[self.AXIS_LEFT_STICK_Y])
        steering = self._apply_deadzone(axes[self.AXIS_RIGHT_STICK_X])

        # Triggers: RT for boost, LT for brake
        # Triggers go from 1.0 (not pressed) to -1.0 (fully pressed)
        rt = (1.0 - axes[self.AXIS_RT]) / 2.0  # 0 to 1
        lt = (1.0 - axes[self.AXIS_LT]) / 2.0  # 0 to 1

        # Calculate speed with boost/brake
        speed_multiplier = self.speed_mode
        if rt > 0.1:
            speed_multiplier *= (1.0 + rt * 0.5)  # Up to 50% boost from trigger
        if lt > 0.1:
            speed_multiplier *= (1.0 - lt * 0.8)  # Up to 80% reduction from brake

        # Create twist message
        cmd = Twist()
        cmd.linear.x = forward * self.max_speed * speed_multiplier
        cmd.angular.z = steering * self.max_angular * speed_multiplier

        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        """Publish zero velocity."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish stop before shutting down
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
