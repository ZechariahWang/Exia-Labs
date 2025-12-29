#!/usr/bin/env python3
"""
Ackermann Hardware Abstraction Layer (HAL) - Base Interface

This module defines the abstract interface for the three-motor Ackermann drive system.
All HAL implementations (simulation, real hardware) must implement this interface.

Three-Motor Architecture:
  1. Steering - Controls front wheel angle (servo/motor)
  2. Throttle - Controls rear wheel drive (motor/ESC)
  3. Brake    - Controls brake actuator (servo/motor/caliper)

Author: Zechariah Wang
Date: December 2025
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable
import math


class DriveMode(Enum):
    PARK = 0      # Brakes engaged, no throttle
    FORWARD = 1   # Normal forward driving
    REVERSE = 2   # Reverse driving
    NEUTRAL = 3   # No brakes, no throttle (coasting)


@dataclass
class AckermannCommand:
    """
    High-level Ackermann drive command.

    This is the interface between the control layer and the HAL.
    All values are in SI units.
    """
    steering_angle: float = 0.0     # Radians, positive = left turn
    throttle: float = 0.0           # 0.0 to 1.0, throttle percentage
    brake: float = 0.0              # 0.0 to 1.0, brake engagement
    drive_mode: DriveMode = DriveMode.NEUTRAL

    def __post_init__(self):
        """Validate and clamp values."""
        self.throttle = max(0.0, min(1.0, self.throttle))
        self.brake = max(0.0, min(1.0, self.brake))


@dataclass
class AckermannState:
    """
    Current state feedback from the Ackermann drive system.

    Provides feedback from sensors/encoders for closed-loop control.
    """
    # Steering feedback
    steering_angle: float = 0.0         # Current steering angle (rad)
    steering_velocity: float = 0.0      # Steering rate (rad/s)

    # Throttle/velocity feedback
    wheel_velocity_left: float = 0.0    # Left rear wheel (rad/s)
    wheel_velocity_right: float = 0.0   # Right rear wheel (rad/s)
    linear_velocity: float = 0.0        # Computed linear velocity (m/s)

    # Brake feedback
    brake_position: float = 0.0         # Brake engagement level (0-1)
    brake_engaged: bool = False         # True if brake is engaged

    # System status
    is_emergency_stopped: bool = False
    fault_code: int = 0                 # 0 = no fault


@dataclass
class AckermannConfig:
    """
    Configuration parameters for the Ackermann drive system.

    These parameters define the physical characteristics and limits
    of the vehicle. Must match the actual robot/ATV specifications.
    """
    # Geometry (meters)
    wheelbase: float = 0.4              # Front to rear axle distance
    track_width: float = 0.45           # Left to right wheel distance
    wheel_radius: float = 0.1           # Wheel radius

    # Steering limits
    max_steering_angle: float = 0.6     # Maximum steering angle (rad)
    steering_rate_limit: float = 2.0    # Max steering rate (rad/s)

    # Speed limits
    max_speed: float = 5.0              # Maximum linear speed (m/s)
    max_wheel_velocity: float = 50.0    # Maximum wheel velocity (rad/s)

    # Acceleration limits
    max_acceleration: float = 2.0       # m/s^2
    max_deceleration: float = 5.0       # m/s^2 (braking)

    # Calibration offsets
    steering_trim: float = 0.0          # Steering center offset (rad)
    throttle_deadband: float = 0.05     # Throttle deadband (0-1)
    brake_deadband: float = 0.02        # Brake deadband (0-1)

    def speed_to_wheel_velocity(self, speed: float) -> float:
        """Convert linear speed (m/s) to wheel angular velocity (rad/s)."""
        return speed / self.wheel_radius

    def wheel_velocity_to_speed(self, wheel_vel: float) -> float:
        """Convert wheel angular velocity (rad/s) to linear speed (m/s)."""
        return wheel_vel * self.wheel_radius


class AckermannHAL(ABC):
    """
    Abstract Hardware Abstraction Layer for Ackermann drive system.

    This interface defines the contract between high-level control logic
    and low-level actuator control. Implementations handle the specifics
    of simulation vs real hardware.

    Usage:
        hal = SimulationHAL(config, node)  # or HardwareHAL(config, node)
        hal.initialize()

        # In control loop:
        hal.set_command(AckermannCommand(steering=0.1, throttle=0.5))
        state = hal.get_state()

        # Cleanup:
        hal.shutdown()
    """

    def __init__(self, config: AckermannConfig):
        """
        Initialize the HAL with configuration.

        Args:
            config: AckermannConfig with vehicle parameters
        """
        self.config = config
        self._initialized = False
        self._emergency_stop = False
        self._state_callback: Optional[Callable[[AckermannState], None]] = None

    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the hardware interface.

        Returns:
            True if initialization successful, False otherwise
        """
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """
        Safely shutdown the hardware interface.

        Should engage brakes and disable throttle before shutdown.
        """
        pass

    @abstractmethod
    def set_command(self, command: AckermannCommand) -> bool:
        """
        Send command to the actuators.

        Args:
            command: AckermannCommand with desired steering, throttle, brake

        Returns:
            True if command was accepted, False otherwise
        """
        pass

    @abstractmethod
    def get_state(self) -> AckermannState:
        """
        Get current state from sensors/encoders.

        Returns:
            AckermannState with current feedback values
        """
        pass

    @abstractmethod
    def emergency_stop(self) -> None:
        """
        Trigger emergency stop.

        Should immediately:
        - Set throttle to zero
        - Engage brakes fully
        - Center steering (if safe)
        """
        pass

    @abstractmethod
    def clear_emergency_stop(self) -> bool:
        """
        Clear emergency stop condition.

        Returns:
            True if e-stop was cleared, False if conditions not met
        """
        pass

    def set_state_callback(self, callback: Callable[[AckermannState], None]) -> None:
        """
        Register callback for state updates.

        Args:
            callback: Function to call when state is updated
        """
        self._state_callback = callback

    def is_initialized(self) -> bool:
        """Check if HAL is initialized."""
        return self._initialized

    def is_emergency_stopped(self) -> bool:
        """Check if emergency stop is active."""
        return self._emergency_stop

    # ==================== Ackermann Geometry Helpers ====================

    def compute_ackermann_angles(self, steering_angle: float) -> tuple:
        """
        Compute inner and outer wheel angles for proper Ackermann geometry.

        For a given "virtual" center steering angle, computes the actual
        angles needed for the inner and outer wheels to avoid tire scrub.

        Args:
            steering_angle: Desired steering angle (rad), positive = left turn

        Returns:
            Tuple of (left_angle, right_angle) in radians
        """
        if abs(steering_angle) < 0.001:
            return (0.0, 0.0)

        L = self.config.wheelbase
        W = self.config.track_width

        # Calculate turning radius for center of rear axle
        # R = L / tan(steering_angle)
        tan_steer = math.tan(abs(steering_angle))

        if tan_steer < 0.001:
            return (steering_angle, steering_angle)

        R = L / tan_steer

        # Inner wheel (tighter turn)
        R_inner = R - W / 2
        inner_angle = math.atan(L / R_inner) if R_inner > 0.01 else self.config.max_steering_angle

        # Outer wheel (wider turn)
        R_outer = R + W / 2
        outer_angle = math.atan(L / R_outer)

        # Clamp to limits
        inner_angle = min(inner_angle, self.config.max_steering_angle)
        outer_angle = min(outer_angle, self.config.max_steering_angle)

        # Apply sign based on turn direction
        if steering_angle > 0:
            # Left turn: left wheel is inner, right wheel is outer
            return (inner_angle, outer_angle)
        else:
            # Right turn: right wheel is inner, left wheel is outer
            return (-outer_angle, -inner_angle)

    def compute_turning_radius(self, steering_angle: float) -> float:
        """
        Compute the turning radius for a given steering angle.

        Args:
            steering_angle: Steering angle in radians

        Returns:
            Turning radius in meters (infinity if going straight)
        """
        if abs(steering_angle) < 0.001:
            return float('inf')

        return self.config.wheelbase / math.tan(abs(steering_angle))

    def compute_max_safe_speed(self, steering_angle: float,
                                lateral_accel_limit: float = 3.0) -> float:
        """
        Compute maximum safe speed for a given steering angle.

        Limits speed based on lateral acceleration to prevent rollover/skid.

        Args:
            steering_angle: Current steering angle (rad)
            lateral_accel_limit: Maximum lateral acceleration (m/s^2)

        Returns:
            Maximum safe speed (m/s)
        """
        R = self.compute_turning_radius(steering_angle)

        if R == float('inf'):
            return self.config.max_speed

        # v_max = sqrt(a_lat * R)
        v_max = math.sqrt(lateral_accel_limit * R)

        return min(v_max, self.config.max_speed)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value to a range."""
    return max(min_val, min(max_val, value))


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation between a and b."""
    return a + (b - a) * clamp(t, 0.0, 1.0)


def rate_limit(current: float, target: float, max_rate: float, dt: float) -> float:
    """Apply rate limiting to a value."""
    max_change = max_rate * dt
    diff = target - current

    if abs(diff) <= max_change:
        return target
    else:
        return current + max_change if diff > 0 else current - max_change
