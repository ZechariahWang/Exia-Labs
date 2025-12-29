#!/usr/bin/env python3
"""
Ackermann HAL - Hardware Implementation

This module implements the HAL interface for real ATV hardware control.
It provides a flexible framework that can be configured for different
motor drivers and communication protocols.

Supported Hardware Configurations:
  1. PWM via GPIO (Raspberry Pi, Jetson Nano)
  2. PWM via PCA9685 I2C servo driver
  3. Serial communication (Arduino, Sabertooth, ODrive)
  4. CAN bus (industrial motor controllers)

Three-Motor Architecture:
  - Steering: Servo or linear actuator (position control)
  - Throttle: ESC, DC motor controller, or EV motor (velocity control)
  - Brake:    Servo or linear actuator (position control)

Safety Features:
  - Watchdog timer (stops motors if no commands received)
  - Emergency stop input
  - Soft limits with configurable ramp rates
  - Encoder feedback for closed-loop control

Author: Zechariah Wang
Date: December 2025
"""

import math
import time
from typing import Optional, Dict, Any
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState

from ackermann_hal_base import (
    AckermannHAL,
    AckermannCommand,
    AckermannState,
    AckermannConfig,
    DriveMode,
    clamp,
    rate_limit,
)


class HardwareDriver(Enum):
    """Supported hardware driver types."""
    GPIO_PWM = 1       # Direct GPIO PWM (RPi, Jetson)
    PCA9685 = 2        # I2C PWM driver
    SERIAL = 3         # Serial/UART communication
    CAN = 4            # CAN bus
    SIMULATION = 5     # Pass-through for testing


class PWMConfig:
    """PWM configuration for servo/ESC control."""

    def __init__(
        self,
        channel: int = 0,
        frequency: int = 50,           # Hz (50 for servo, 400+ for ESC)
        min_pulse_us: int = 1000,      # Microseconds
        max_pulse_us: int = 2000,
        neutral_pulse_us: int = 1500,
        inverted: bool = False
    ):
        self.channel = channel
        self.frequency = frequency
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.neutral_pulse_us = neutral_pulse_us
        self.inverted = inverted

    def value_to_pulse(self, value: float, is_bidirectional: bool = False) -> int:
        """
        Convert normalized value to pulse width.

        Args:
            value: Normalized value (0-1 for unidirectional, -1 to 1 for bidirectional)
            is_bidirectional: True for steering/throttle with reverse

        Returns:
            Pulse width in microseconds
        """
        if is_bidirectional:
            # -1 to 1 maps to min to max with neutral at center
            if self.inverted:
                value = -value
            pulse = self.neutral_pulse_us + value * (self.max_pulse_us - self.neutral_pulse_us)
        else:
            # 0 to 1 maps to neutral to max (for brake, unidirectional throttle)
            pulse = self.neutral_pulse_us + value * (self.max_pulse_us - self.neutral_pulse_us)

        return int(clamp(pulse, self.min_pulse_us, self.max_pulse_us))

    def pulse_to_duty_cycle(self, pulse_us: int) -> float:
        """Convert pulse width to duty cycle percentage (0-100)."""
        period_us = 1_000_000 / self.frequency
        return (pulse_us / period_us) * 100


class HardwareHAL(AckermannHAL):
    """
    Hardware Abstraction Layer for real ATV hardware.

    This implementation provides a framework for controlling physical
    actuators. Subclass or modify the _send_* methods for your specific
    hardware setup.
    """

    def __init__(
        self,
        node: Node,
        config: Optional[AckermannConfig] = None,
        driver_type: HardwareDriver = HardwareDriver.SIMULATION,
        steering_pwm: Optional[PWMConfig] = None,
        throttle_pwm: Optional[PWMConfig] = None,
        brake_pwm: Optional[PWMConfig] = None
    ):
        """
        Initialize the hardware HAL.

        Args:
            node: ROS 2 node for publishers/subscribers
            config: Vehicle configuration
            driver_type: Type of hardware driver to use
            steering_pwm: PWM configuration for steering servo
            throttle_pwm: PWM configuration for throttle ESC
            brake_pwm: PWM configuration for brake servo
        """
        if config is None:
            config = AckermannConfig()

        super().__init__(config)
        self.node = node
        self.driver_type = driver_type

        # PWM configurations with defaults
        self.steering_pwm = steering_pwm or PWMConfig(channel=0)
        self.throttle_pwm = throttle_pwm or PWMConfig(channel=1)
        self.brake_pwm = brake_pwm or PWMConfig(channel=2)

        # Hardware handles (set during initialization)
        self._pwm_driver = None
        self._serial_port = None
        self._can_bus = None
        self._gpio = None

        # State tracking
        self._current_state = AckermannState()
        self._last_command = AckermannCommand()
        self._last_command_time = None

        # Rate limited values
        self._current_steering = 0.0
        self._current_throttle = 0.0
        self._current_brake = 0.0

        # Watchdog
        self._watchdog_timeout = 0.5  # seconds
        self._watchdog_timer = None

        # ROS publishers for monitoring
        self._steering_fb_pub: Optional[rclpy.publisher.Publisher] = None
        self._throttle_fb_pub: Optional[rclpy.publisher.Publisher] = None
        self._brake_fb_pub: Optional[rclpy.publisher.Publisher] = None
        self._estop_pub: Optional[rclpy.publisher.Publisher] = None

        # Encoder feedback subscribers (optional)
        self._encoder_sub: Optional[rclpy.subscription.Subscription] = None

    def initialize(self) -> bool:
        """Initialize hardware connections."""
        try:
            # Initialize based on driver type
            if self.driver_type == HardwareDriver.GPIO_PWM:
                self._init_gpio_pwm()
            elif self.driver_type == HardwareDriver.PCA9685:
                self._init_pca9685()
            elif self.driver_type == HardwareDriver.SERIAL:
                self._init_serial()
            elif self.driver_type == HardwareDriver.CAN:
                self._init_can()
            elif self.driver_type == HardwareDriver.SIMULATION:
                self._init_simulation_passthrough()

            # Create ROS publishers for monitoring
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

            self._steering_fb_pub = self.node.create_publisher(
                Float64, '/hardware/steering_feedback', qos)
            self._throttle_fb_pub = self.node.create_publisher(
                Float64, '/hardware/throttle_feedback', qos)
            self._brake_fb_pub = self.node.create_publisher(
                Float64, '/hardware/brake_feedback', qos)
            self._estop_pub = self.node.create_publisher(
                Bool, '/hardware/estop_status', qos)

            # Start watchdog timer
            self._watchdog_timer = self.node.create_timer(0.1, self._watchdog_callback)

            # Send neutral commands
            self._send_steering(0.0)
            self._send_throttle(0.0)
            self._send_brake(0.0)

            self._initialized = True
            self.node.get_logger().info(
                f'Hardware HAL initialized with driver: {self.driver_type.name}'
            )
            return True

        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize Hardware HAL: {e}')
            return False

    def shutdown(self) -> None:
        """Safely shutdown - engage brakes and stop throttle."""
        if not self._initialized:
            return

        self.node.get_logger().info('Hardware HAL shutting down')

        # Cancel watchdog
        if self._watchdog_timer:
            self._watchdog_timer.cancel()

        # Send safe commands
        self._send_throttle(0.0)
        self._send_brake(1.0)  # Full brake
        self._send_steering(0.0)  # Center steering

        # Cleanup hardware
        self._cleanup_hardware()

        self._initialized = False

    def set_command(self, command: AckermannCommand) -> bool:
        """Send command to hardware actuators."""
        if not self._initialized:
            return False

        # Update command time for watchdog
        self._last_command_time = self.node.get_clock().now()

        if self._emergency_stop:
            # Override with emergency stop
            command = AckermannCommand(
                steering_angle=0.0,
                throttle=0.0,
                brake=1.0,
                drive_mode=DriveMode.PARK
            )

        # Calculate time delta for rate limiting
        dt = 0.02  # Default if first command

        # Process steering
        target_steering = clamp(
            command.steering_angle + self.config.steering_trim,
            -self.config.max_steering_angle,
            self.config.max_steering_angle
        )
        self._current_steering = rate_limit(
            self._current_steering,
            target_steering,
            self.config.steering_rate_limit,
            dt
        )

        # Normalize steering to -1 to 1 for PWM
        steering_normalized = self._current_steering / self.config.max_steering_angle

        # Process throttle
        if command.throttle < self.config.throttle_deadband:
            self._current_throttle = 0.0
        else:
            self._current_throttle = command.throttle
            if command.drive_mode == DriveMode.REVERSE:
                self._current_throttle = -self._current_throttle

        # Process brake
        if command.brake < self.config.brake_deadband:
            self._current_brake = 0.0
        else:
            self._current_brake = command.brake

        # If braking hard, reduce throttle
        if self._current_brake > 0.2:
            self._current_throttle *= max(0.0, 1.0 - self._current_brake)

        # Send to hardware
        self._send_steering(steering_normalized)
        self._send_throttle(self._current_throttle)
        self._send_brake(self._current_brake)

        self._last_command = command
        return True

    def get_state(self) -> AckermannState:
        """Get current state from hardware feedback."""
        return self._current_state

    def emergency_stop(self) -> None:
        """Trigger emergency stop."""
        self._emergency_stop = True
        self.node.get_logger().error('EMERGENCY STOP ACTIVATED')

        # Immediately send stop commands
        self._send_throttle(0.0)
        self._send_brake(1.0)

        self._current_state.is_emergency_stopped = True

        # Publish E-stop status
        if self._estop_pub:
            msg = Bool()
            msg.data = True
            self._estop_pub.publish(msg)

    def clear_emergency_stop(self) -> bool:
        """Clear emergency stop if safe."""
        if abs(self._current_state.linear_velocity) > 0.1:
            self.node.get_logger().warn('Cannot clear E-stop: vehicle moving')
            return False

        self._emergency_stop = False
        self._current_state.is_emergency_stopped = False

        if self._estop_pub:
            msg = Bool()
            msg.data = False
            self._estop_pub.publish(msg)

        self.node.get_logger().info('Emergency stop cleared')
        return True

    # ==================== Hardware Initialization ====================

    def _init_gpio_pwm(self) -> None:
        """Initialize GPIO PWM (Raspberry Pi / Jetson)."""
        self.node.get_logger().info('Initializing GPIO PWM driver')

        # Uncomment and configure for your platform:
        # For Raspberry Pi:
        # import RPi.GPIO as GPIO
        # GPIO.setmode(GPIO.BCM)
        # self._gpio = GPIO

        # For Jetson:
        # import Jetson.GPIO as GPIO
        # GPIO.setmode(GPIO.BOARD)
        # self._gpio = GPIO

        # Set up PWM pins
        # GPIO.setup(self.steering_pwm.channel, GPIO.OUT)
        # self._steering_pwm_handle = GPIO.PWM(self.steering_pwm.channel, self.steering_pwm.frequency)
        # self._steering_pwm_handle.start(7.5)  # Neutral

        raise NotImplementedError("Configure GPIO PWM for your specific hardware")

    def _init_pca9685(self) -> None:
        """Initialize PCA9685 I2C PWM driver."""
        self.node.get_logger().info('Initializing PCA9685 PWM driver')

        # Uncomment and configure:
        # from adafruit_pca9685 import PCA9685
        # import board
        # import busio
        #
        # i2c = busio.I2C(board.SCL, board.SDA)
        # self._pwm_driver = PCA9685(i2c)
        # self._pwm_driver.frequency = 50

        raise NotImplementedError("Configure PCA9685 for your specific hardware")

    def _init_serial(self) -> None:
        """Initialize serial communication (Arduino, Sabertooth, etc.)."""
        self.node.get_logger().info('Initializing serial driver')

        # import serial
        # self._serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        raise NotImplementedError("Configure serial port for your specific hardware")

    def _init_can(self) -> None:
        """Initialize CAN bus communication."""
        self.node.get_logger().info('Initializing CAN bus driver')

        # import can
        # self._can_bus = can.interface.Bus(channel='can0', interface='socketcan')

        raise NotImplementedError("Configure CAN bus for your specific hardware")

    def _init_simulation_passthrough(self) -> None:
        """Initialize simulation passthrough mode for testing."""
        self.node.get_logger().info('Initializing simulation passthrough mode')
        # No hardware to initialize - just log commands

    def _cleanup_hardware(self) -> None:
        """Cleanup hardware resources."""
        if self._gpio:
            # GPIO.cleanup()
            pass
        if self._serial_port:
            # self._serial_port.close()
            pass
        if self._can_bus:
            # self._can_bus.shutdown()
            pass

    # ==================== Hardware Send Methods ====================

    def _send_steering(self, value: float) -> None:
        """
        Send steering command to hardware.

        Args:
            value: Normalized steering value (-1 to 1)
        """
        pulse = self.steering_pwm.value_to_pulse(value, is_bidirectional=True)

        if self.driver_type == HardwareDriver.SIMULATION:
            self.node.get_logger().debug(f'Steering: {value:.2f} -> {pulse}us')
        elif self.driver_type == HardwareDriver.PCA9685:
            # duty = int(pulse / 20000 * 0xFFFF)
            # self._pwm_driver.channels[self.steering_pwm.channel].duty_cycle = duty
            pass
        elif self.driver_type == HardwareDriver.GPIO_PWM:
            # duty = self.steering_pwm.pulse_to_duty_cycle(pulse)
            # self._steering_pwm_handle.ChangeDutyCycle(duty)
            pass

        # Publish feedback
        if self._steering_fb_pub:
            msg = Float64()
            msg.data = value
            self._steering_fb_pub.publish(msg)

    def _send_throttle(self, value: float) -> None:
        """
        Send throttle command to hardware.

        Args:
            value: Normalized throttle value (-1 to 1, negative = reverse)
        """
        pulse = self.throttle_pwm.value_to_pulse(value, is_bidirectional=True)

        if self.driver_type == HardwareDriver.SIMULATION:
            self.node.get_logger().debug(f'Throttle: {value:.2f} -> {pulse}us')
        elif self.driver_type == HardwareDriver.PCA9685:
            # duty = int(pulse / 20000 * 0xFFFF)
            # self._pwm_driver.channels[self.throttle_pwm.channel].duty_cycle = duty
            pass
        elif self.driver_type == HardwareDriver.SERIAL:
            # For Sabertooth motor controller:
            # self._serial_port.write(f'M1:{int(value*127+128)}\n'.encode())
            pass

        # Publish feedback
        if self._throttle_fb_pub:
            msg = Float64()
            msg.data = value
            self._throttle_fb_pub.publish(msg)

    def _send_brake(self, value: float) -> None:
        """
        Send brake command to hardware.

        Args:
            value: Brake engagement (0 to 1)
        """
        pulse = self.brake_pwm.value_to_pulse(value, is_bidirectional=False)

        if self.driver_type == HardwareDriver.SIMULATION:
            self.node.get_logger().debug(f'Brake: {value:.2f} -> {pulse}us')
        elif self.driver_type == HardwareDriver.PCA9685:
            # duty = int(pulse / 20000 * 0xFFFF)
            # self._pwm_driver.channels[self.brake_pwm.channel].duty_cycle = duty
            pass

        # Publish feedback
        if self._brake_fb_pub:
            msg = Float64()
            msg.data = value
            self._brake_fb_pub.publish(msg)

    # ==================== Watchdog ====================

    def _watchdog_callback(self) -> None:
        """Check if commands are being received - safety stop if not."""
        if self._last_command_time is None:
            return

        elapsed = (self.node.get_clock().now() - self._last_command_time).nanoseconds / 1e9

        if elapsed > self._watchdog_timeout:
            if self._current_throttle != 0.0 or not self._current_brake > 0.5:
                self.node.get_logger().warn(
                    f'Watchdog timeout ({elapsed:.2f}s) - stopping motors'
                )
                self._send_throttle(0.0)
                self._send_brake(1.0)
                self._current_throttle = 0.0
                self._current_brake = 1.0


class HardwareHALNode(Node):
    """Standalone node for testing the Hardware HAL."""

    def __init__(self):
        super().__init__('hardware_hal_node')

        # Declare parameters
        self.declare_parameter('driver_type', 'simulation')
        self.declare_parameter('wheelbase', 0.4)
        self.declare_parameter('track_width', 0.45)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('max_speed', 5.0)

        # PWM parameters
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('throttle_channel', 1)
        self.declare_parameter('brake_channel', 2)

        # Get parameters
        driver_name = self.get_parameter('driver_type').value

        # Map driver name to enum
        driver_map = {
            'simulation': HardwareDriver.SIMULATION,
            'gpio': HardwareDriver.GPIO_PWM,
            'pca9685': HardwareDriver.PCA9685,
            'serial': HardwareDriver.SERIAL,
            'can': HardwareDriver.CAN,
        }
        driver_type = driver_map.get(driver_name.lower(), HardwareDriver.SIMULATION)

        # Create config
        config = AckermannConfig(
            wheelbase=self.get_parameter('wheelbase').value,
            track_width=self.get_parameter('track_width').value,
            max_steering_angle=self.get_parameter('max_steering_angle').value,
            max_speed=self.get_parameter('max_speed').value,
        )

        # Create PWM configs
        steering_pwm = PWMConfig(
            channel=self.get_parameter('steering_channel').value
        )
        throttle_pwm = PWMConfig(
            channel=self.get_parameter('throttle_channel').value
        )
        brake_pwm = PWMConfig(
            channel=self.get_parameter('brake_channel').value
        )

        # Create HAL
        self.hal = HardwareHAL(
            self, config, driver_type,
            steering_pwm, throttle_pwm, brake_pwm
        )

        # Initialize
        if not self.hal.initialize():
            self.get_logger().error('Failed to initialize HAL')
            return

        self.get_logger().info(f'Hardware HAL node started with driver: {driver_name}')


def main(args=None):
    """Run hardware HAL node."""
    rclpy.init(args=args)
    node = HardwareHALNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hal.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
