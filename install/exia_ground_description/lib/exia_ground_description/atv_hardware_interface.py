#!/usr/bin/env python3
"""
ATV Hardware Interface - Motor Driver Bridge.

This node subscribes to the ATV controller outputs and sends
commands to actual motor controllers.

MODIFY THIS FILE for your specific hardware setup:
- PWM via GPIO (Raspberry Pi, Jetson)
- Serial communication (Arduino, motor controllers)
- CAN bus (industrial motor controllers)
- RC ESC/Servo via PCA9685

Example hardware configurations:
1. RC-style ESC + Servo:
   - Throttle: ESC on PWM channel (1000-2000us)
   - Brake: Servo or second ESC channel
   - Steering: Servo on PWM channel (1000-2000us)

2. Industrial motor controllers:
   - CAN bus or serial commands
   - Velocity/position feedback

Subscribes:
  - /atv/throttle (std_msgs/Float64) 0.0-1.0
  - /atv/brake (std_msgs/Float64) 0.0-1.0
  - /atv/steering (std_msgs/Float64) -1.0 to 1.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# ==================== HARDWARE IMPORTS ====================
# Uncomment the imports you need for your hardware:

# For Raspberry Pi GPIO PWM:
# import RPi.GPIO as GPIO

# For Jetson GPIO:
# import Jetson.GPIO as GPIO

# For PCA9685 PWM driver (I2C servo/ESC controller):
# from adafruit_pca9685 import PCA9685
# import board
# import busio

# For serial communication (Arduino, etc):
# import serial


class ATVHardwareInterface(Node):
    def __init__(self):
        super().__init__('atv_hardware_interface')

        # ==================== PARAMETERS ====================
        self.declare_parameter('throttle_pin', 18)      # GPIO pin or PWM channel
        self.declare_parameter('brake_pin', 19)
        self.declare_parameter('steering_pin', 20)
        self.declare_parameter('pwm_frequency', 50)     # Hz (50 for servos/ESC)
        self.declare_parameter('pwm_min_us', 1000)      # Microseconds
        self.declare_parameter('pwm_max_us', 2000)
        self.declare_parameter('pwm_neutral_us', 1500)
        self.declare_parameter('use_simulation', True)  # Set False for real hardware

        self.use_simulation = self.get_parameter('use_simulation').value

        # ==================== HARDWARE SETUP ====================
        if not self.use_simulation:
            self.setup_hardware()
        else:
            self.get_logger().info('Running in SIMULATION mode - no hardware output')

        # ==================== SUBSCRIBERS ====================
        self.throttle_sub = self.create_subscription(
            Float64, '/atv/throttle', self.throttle_callback, 10)
        self.brake_sub = self.create_subscription(
            Float64, '/atv/brake', self.brake_callback, 10)
        self.steering_sub = self.create_subscription(
            Float64, '/atv/steering', self.steering_callback, 10)

        # ==================== STATE ====================
        self.current_throttle = 0.0
        self.current_brake = 0.0
        self.current_steering = 0.0

        # Safety watchdog - stop if no commands received
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info('ATV Hardware Interface started')

    def setup_hardware(self):
        """Initialize hardware connections."""
        self.get_logger().info('Initializing hardware...')

        # ==================== EXAMPLE: GPIO PWM ====================
        # Uncomment and modify for your setup:
        #
        # GPIO.setmode(GPIO.BCM)
        # self.throttle_pin = self.get_parameter('throttle_pin').value
        # self.brake_pin = self.get_parameter('brake_pin').value
        # self.steering_pin = self.get_parameter('steering_pin').value
        #
        # GPIO.setup(self.throttle_pin, GPIO.OUT)
        # GPIO.setup(self.brake_pin, GPIO.OUT)
        # GPIO.setup(self.steering_pin, GPIO.OUT)
        #
        # freq = self.get_parameter('pwm_frequency').value
        # self.throttle_pwm = GPIO.PWM(self.throttle_pin, freq)
        # self.brake_pwm = GPIO.PWM(self.brake_pin, freq)
        # self.steering_pwm = GPIO.PWM(self.steering_pin, freq)
        #
        # self.throttle_pwm.start(7.5)  # Neutral
        # self.brake_pwm.start(7.5)
        # self.steering_pwm.start(7.5)

        # ==================== EXAMPLE: PCA9685 ====================
        # i2c = busio.I2C(board.SCL, board.SDA)
        # self.pca = PCA9685(i2c)
        # self.pca.frequency = 50

        # ==================== EXAMPLE: Serial/Arduino ====================
        # self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        self.get_logger().info('Hardware initialized')

    def value_to_pwm_duty(self, value, is_steering=False):
        """Convert -1.0 to 1.0 (or 0.0 to 1.0) to PWM duty cycle."""
        pwm_min = self.get_parameter('pwm_min_us').value
        pwm_max = self.get_parameter('pwm_max_us').value
        pwm_neutral = self.get_parameter('pwm_neutral_us').value

        if is_steering:
            # Steering: -1.0 to 1.0 maps to min to max
            pwm_us = pwm_neutral + (value * (pwm_max - pwm_neutral))
        else:
            # Throttle/brake: 0.0 to 1.0 maps to neutral to max
            pwm_us = pwm_neutral + (value * (pwm_max - pwm_neutral))

        # Convert microseconds to duty cycle (for 50Hz: 20ms period)
        # duty_cycle = (pwm_us / 20000) * 100
        duty_cycle = (pwm_us / 20000.0) * 100.0

        return max(2.5, min(12.5, duty_cycle))  # Clamp to safe range

    def throttle_callback(self, msg: Float64):
        self.current_throttle = max(0.0, min(1.0, msg.data))
        self.last_cmd_time = self.get_clock().now()

        if self.use_simulation:
            self.get_logger().debug(f'Throttle: {self.current_throttle:.2f}')
        else:
            self.send_throttle(self.current_throttle)

    def brake_callback(self, msg: Float64):
        self.current_brake = max(0.0, min(1.0, msg.data))
        self.last_cmd_time = self.get_clock().now()

        if self.use_simulation:
            self.get_logger().debug(f'Brake: {self.current_brake:.2f}')
        else:
            self.send_brake(self.current_brake)

    def steering_callback(self, msg: Float64):
        self.current_steering = max(-1.0, min(1.0, msg.data))
        self.last_cmd_time = self.get_clock().now()

        if self.use_simulation:
            self.get_logger().debug(f'Steering: {self.current_steering:.2f}')
        else:
            self.send_steering(self.current_steering)

    def send_throttle(self, value):
        """Send throttle command to hardware."""
        duty = self.value_to_pwm_duty(value)
        # self.throttle_pwm.ChangeDutyCycle(duty)
        # OR for PCA9685:
        # self.pca.channels[0].duty_cycle = int(duty / 100 * 0xFFFF)
        pass

    def send_brake(self, value):
        """Send brake command to hardware."""
        duty = self.value_to_pwm_duty(value)
        # self.brake_pwm.ChangeDutyCycle(duty)
        pass

    def send_steering(self, value):
        """Send steering command to hardware."""
        duty = self.value_to_pwm_duty(value, is_steering=True)
        # self.steering_pwm.ChangeDutyCycle(duty)
        pass

    def watchdog_check(self):
        """Safety check - stop motors if no commands received."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > 0.5:  # 500ms timeout
            if self.current_throttle > 0 or abs(self.current_steering) > 0.1:
                self.get_logger().warn('Watchdog timeout - stopping motors')
                self.current_throttle = 0.0
                self.current_brake = 1.0  # Apply brake

                if not self.use_simulation:
                    self.send_throttle(0.0)
                    self.send_brake(1.0)
                    self.send_steering(0.0)

    def shutdown(self):
        """Clean shutdown - stop all motors."""
        self.get_logger().info('Shutting down - stopping motors')

        if not self.use_simulation:
            self.send_throttle(0.0)
            self.send_brake(1.0)
            self.send_steering(0.0)

            # Cleanup GPIO if using
            # GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = ATVHardwareInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
