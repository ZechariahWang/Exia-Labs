"""
Ackermann HAL - Simulation Implementation

Implements the HAL interface for Gazebo Fortress simulation
using ros2_control with ForwardCommandController.

Communication:
    - Steering: /steering_controller/commands (Float64MultiArray)
    - Throttle: /throttle_controller/commands (Float64MultiArray)
    - Brake:    /brake_controller/commands (Float64MultiArray)
    - Feedback: /joint_states (JointState)

Author: Zechariah Wang
Date: December 2025
"""

from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from .base import (
    AckermannHAL,
    AckermannCommand,
    AckermannState,
    AckermannConfig,
    DriveMode,
    clamp,
    rate_limit,
)


class SimulationHAL(AckermannHAL):
    """
    Hardware Abstraction Layer for Gazebo Fortress simulation.

    Uses ros2_control ForwardCommandController for actuator control
    and JointState messages for feedback.
    """

    def __init__(self, node: Node, config: Optional[AckermannConfig] = None):
        """
        Initialize the simulation HAL.

        Args:
            node: ROS 2 node for publishers/subscribers
            config: Optional AckermannConfig, uses defaults if not provided
        """
        if config is None:
            config = AckermannConfig()

        super().__init__(config)
        self.node = node

        # Current state
        self._current_state = AckermannState()
        self._last_command = AckermannCommand()
        self._last_command_time = None

        # Rate limiting state
        self._current_steering = 0.0
        self._current_throttle = 0.0
        self._current_brake = 0.0

        # Publishers and subscribers (created in initialize)
        self._steering_pub = None
        self._throttle_pub = None
        self._brake_pub = None
        self._joint_state_sub = None

        # Joint name mappings
        self._steering_joints = ['front_left_steer_joint', 'front_right_steer_joint']
        self._throttle_joints = ['rear_left_wheel_joint', 'rear_right_wheel_joint']
        self._brake_joint = 'brake_joint'

    def initialize(self) -> bool:
        """Initialize ROS 2 publishers and subscribers."""
        try:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

            # Create publishers for controllers
            self._steering_pub = self.node.create_publisher(
                Float64MultiArray, '/steering_controller/commands', qos)

            self._throttle_pub = self.node.create_publisher(
                Float64MultiArray, '/throttle_controller/commands', qos)

            self._brake_pub = self.node.create_publisher(
                Float64MultiArray, '/brake_controller/commands', qos)

            # Subscribe to joint states for feedback
            self._joint_state_sub = self.node.create_subscription(
                JointState, '/joint_states', self._joint_state_callback, qos)

            self._initialized = True
            self.node.get_logger().info('Simulation HAL initialized successfully')
            return True

        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize Simulation HAL: {e}')
            return False

    def shutdown(self) -> None:
        """Safely shutdown - engage brakes and stop throttle."""
        if not self._initialized:
            return

        self.node.get_logger().info('Simulation HAL shutting down - engaging brakes')
        self._send_throttle(0.0, 0.0)
        self._send_brake(1.0)
        self._send_steering(0.0, 0.0)
        self._initialized = False

    def set_command(self, command: AckermannCommand) -> bool:
        """
        Send command to simulated actuators.

        Applies rate limiting and proper Ackermann geometry.
        """
        if not self._initialized:
            return False

        if self._emergency_stop:
            command = AckermannCommand(
                steering_angle=0.0, throttle=0.0, brake=1.0, drive_mode=DriveMode.PARK)

        # Get time delta for rate limiting
        now = self.node.get_clock().now()
        if self._last_command_time is None:
            dt = 0.02
        else:
            dt = (now - self._last_command_time).nanoseconds / 1e9
        self._last_command_time = now

        # Apply rate limiting to steering
        target_steering = clamp(
            command.steering_angle + self.config.steering_trim,
            -self.config.max_steering_angle,
            self.config.max_steering_angle
        )
        self._current_steering = rate_limit(
            self._current_steering, target_steering,
            self.config.steering_rate_limit, dt
        )

        # Compute proper Ackermann steering angles
        left_angle, right_angle = self.compute_ackermann_angles(self._current_steering)

        # Process throttle with deadband
        if command.throttle < self.config.throttle_deadband:
            self._current_throttle = 0.0
        else:
            target_speed = command.throttle * self.config.max_speed
            target_wheel_vel = self.config.speed_to_wheel_velocity(target_speed)
            if command.drive_mode == DriveMode.REVERSE:
                target_wheel_vel = -target_wheel_vel
            self._current_throttle = target_wheel_vel

        # Process brake with deadband
        if command.brake < self.config.brake_deadband:
            self._current_brake = 0.0
        else:
            self._current_brake = command.brake

        # If braking, reduce throttle
        if self._current_brake > 0.1:
            self._current_throttle *= (1.0 - self._current_brake)

        # Send commands to controllers
        self._send_steering(left_angle, right_angle)
        self._send_throttle(self._current_throttle, self._current_throttle)
        self._send_brake(self._current_brake)

        self._last_command = command
        return True

    def get_state(self) -> AckermannState:
        """Get current state from joint feedback."""
        return self._current_state

    def emergency_stop(self) -> None:
        """Trigger emergency stop."""
        self._emergency_stop = True
        self.node.get_logger().warn('EMERGENCY STOP ACTIVATED')
        self._send_throttle(0.0, 0.0)
        self._send_brake(1.0)
        self._current_state.is_emergency_stopped = True

    def clear_emergency_stop(self) -> bool:
        """Clear emergency stop if safe."""
        if abs(self._current_state.linear_velocity) > 0.1:
            self.node.get_logger().warn('Cannot clear E-stop: vehicle still moving')
            return False

        self._emergency_stop = False
        self._current_state.is_emergency_stopped = False
        self.node.get_logger().info('Emergency stop cleared')
        return True

    # ==================== Private Methods ====================

    def _joint_state_callback(self, msg: JointState) -> None:
        """Process joint state feedback."""
        try:
            positions = dict(zip(msg.name, msg.position))
            velocities = dict(zip(msg.name, msg.velocity))

            # Extract steering feedback
            left_steer_pos = positions.get(self._steering_joints[0], 0.0)
            right_steer_pos = positions.get(self._steering_joints[1], 0.0)
            self._current_state.steering_angle = (left_steer_pos + right_steer_pos) / 2.0

            left_steer_vel = velocities.get(self._steering_joints[0], 0.0)
            right_steer_vel = velocities.get(self._steering_joints[1], 0.0)
            self._current_state.steering_velocity = (left_steer_vel + right_steer_vel) / 2.0

            # Extract wheel velocities
            self._current_state.wheel_velocity_left = velocities.get(
                self._throttle_joints[0], 0.0)
            self._current_state.wheel_velocity_right = velocities.get(
                self._throttle_joints[1], 0.0)

            # Compute linear velocity
            avg_wheel_vel = (
                self._current_state.wheel_velocity_left +
                self._current_state.wheel_velocity_right) / 2.0
            self._current_state.linear_velocity = self.config.wheel_velocity_to_speed(avg_wheel_vel)

            # Extract brake feedback
            self._current_state.brake_position = positions.get(self._brake_joint, 0.0)
            self._current_state.brake_engaged = self._current_state.brake_position > 0.1

            # Invoke callback if registered
            if self._state_callback is not None:
                self._state_callback(self._current_state)

        except Exception as e:
            self.node.get_logger().debug(f'Error processing joint states: {e}')

    def _send_steering(self, left_angle: float, right_angle: float) -> None:
        """Send steering command to controller."""
        if self._steering_pub is None:
            return
        msg = Float64MultiArray()
        msg.data = [left_angle, right_angle]
        self._steering_pub.publish(msg)

    def _send_throttle(self, left_vel: float, right_vel: float) -> None:
        """Send throttle command to controller."""
        if self._throttle_pub is None:
            return
        msg = Float64MultiArray()
        msg.data = [left_vel, right_vel]
        self._throttle_pub.publish(msg)

    def _send_brake(self, brake_level: float) -> None:
        """Send brake command to controller."""
        if self._brake_pub is None:
            return
        msg = Float64MultiArray()
        msg.data = [clamp(brake_level, 0.0, 1.0)]
        self._brake_pub.publish(msg)
