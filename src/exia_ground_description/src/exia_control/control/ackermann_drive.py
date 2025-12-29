"""
Ackermann Drive Controller

Converts high-level velocity commands (cmd_vel) to low-level
three-motor commands (steering, throttle, brake) using Ackermann kinematics.

Author: Zechariah Wang
Date: December 2025
"""

import math
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Twist

from ..hal.base import (
    AckermannCommand,
    AckermannConfig,
    AckermannState,
    DriveMode,
    clamp,
)


@dataclass
class DriveControllerConfig:
    """Configuration for the drive controller."""
    # Kinematic parameters
    wheelbase: float = 1.3
    track_width: float = 1.1
    wheel_radius: float = 0.3

    # Limits
    max_steering_angle: float = 0.6
    max_speed: float = 5.0
    max_acceleration: float = 2.0
    max_deceleration: float = 5.0

    # Safety
    lateral_accel_limit: float = 3.0  # m/s^2
    cmd_timeout: float = 0.5  # seconds
    auto_brake_on_stop: bool = True


class AckermannDriveController:
    """
    Converts Twist commands to Ackermann drive commands.

    Uses Ackermann kinematics to compute proper steering angle
    from linear and angular velocity commands.
    """

    def __init__(self, config: Optional[DriveControllerConfig] = None):
        """
        Initialize the drive controller.

        Args:
            config: DriveControllerConfig, uses defaults if not provided
        """
        self.config = config or DriveControllerConfig()

        # Current command state
        self._last_cmd_time: Optional[float] = None
        self._current_speed = 0.0
        self._current_steering = 0.0

    def cmd_vel_to_ackermann(self, cmd: Twist, current_time: float) -> AckermannCommand:
        """
        Convert Twist (cmd_vel) to AckermannCommand.

        Args:
            cmd: Twist message with linear.x and angular.z
            current_time: Current time in seconds

        Returns:
            AckermannCommand for the HAL
        """
        # Check for timeout
        if self._last_cmd_time is not None:
            dt = current_time - self._last_cmd_time
            if dt > self.config.cmd_timeout:
                # Command timeout - stop the vehicle
                return AckermannCommand(
                    steering_angle=0.0,
                    throttle=0.0,
                    brake=1.0 if self.config.auto_brake_on_stop else 0.0,
                    drive_mode=DriveMode.PARK
                )
        self._last_cmd_time = current_time

        linear_vel = cmd.linear.x
        angular_vel = cmd.angular.z

        # Determine drive mode
        if abs(linear_vel) < 0.01 and abs(angular_vel) < 0.01:
            # Stopped
            return AckermannCommand(
                steering_angle=self._current_steering,  # Hold current steering
                throttle=0.0,
                brake=0.3 if self.config.auto_brake_on_stop else 0.0,
                drive_mode=DriveMode.PARK
            )

        # Determine direction
        if linear_vel >= 0:
            drive_mode = DriveMode.FORWARD
            speed = linear_vel
        else:
            drive_mode = DriveMode.REVERSE
            speed = -linear_vel

        # Compute steering angle from angular velocity
        # For Ackermann: omega = v * tan(delta) / L
        # Therefore: delta = atan(omega * L / v)
        if abs(speed) > 0.01:
            # Normal case: compute steering from kinematics
            steering_angle = math.atan2(
                angular_vel * self.config.wheelbase,
                speed
            )
        else:
            # Low speed: limit steering based on angular velocity
            steering_angle = clamp(
                angular_vel * 0.5,  # Simple scaling at low speed
                -self.config.max_steering_angle,
                self.config.max_steering_angle
            )

        # Apply steering limits
        steering_angle = clamp(
            steering_angle,
            -self.config.max_steering_angle,
            self.config.max_steering_angle
        )

        # Apply speed-dependent steering limit for safety
        max_safe_steering = self._compute_safe_steering(speed)
        steering_angle = clamp(steering_angle, -max_safe_steering, max_safe_steering)

        # Convert speed to throttle percentage
        throttle = clamp(speed / self.config.max_speed, 0.0, 1.0)

        # Update state
        self._current_speed = speed
        self._current_steering = steering_angle

        return AckermannCommand(
            steering_angle=steering_angle,
            throttle=throttle,
            brake=0.0,
            drive_mode=drive_mode
        )

    def _compute_safe_steering(self, speed: float) -> float:
        """
        Compute maximum safe steering angle for current speed.

        Prevents excessive lateral acceleration that could cause rollover.

        Args:
            speed: Current speed in m/s

        Returns:
            Maximum safe steering angle in radians
        """
        if speed < 0.1:
            return self.config.max_steering_angle

        # Max steering from lateral acceleration limit
        # a_lat = v^2 / R, R = L / tan(delta)
        # a_lat = v^2 * tan(delta) / L
        # tan(delta) = a_lat * L / v^2
        max_tan = self.config.lateral_accel_limit * self.config.wheelbase / (speed * speed)
        max_steering = math.atan(max_tan)

        return min(max_steering, self.config.max_steering_angle)

    def get_current_state(self) -> dict:
        """Get current controller state for debugging."""
        return {
            'speed': self._current_speed,
            'steering': self._current_steering,
            'steering_deg': math.degrees(self._current_steering),
        }

    def reset(self):
        """Reset controller state."""
        self._last_cmd_time = None
        self._current_speed = 0.0
        self._current_steering = 0.0
