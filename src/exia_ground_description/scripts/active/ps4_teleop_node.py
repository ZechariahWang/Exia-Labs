#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import termios
import time

SERIAL_ERRORS = (serial.SerialException, OSError, termios.error)

class PS4TeleopNode(Node):
    AXIS_LEFT_STICK_X = 0
    AXIS_L2 = 3
    AXIS_R2 = 4
    BUTTON_CROSS = 0
    BUTTON_CIRCLE = 1

    def __init__(self):
        super().__init__('ps4_teleop_simple')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.enabled = True
        self._prev_buttons = []

        self._steer = 90
        self._throttle = 90
        self._brake = 0

        self.serial_conn = None
        self._init_serial()
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.create_timer(0.05, self._send_callback)
        self.get_logger().info('PS4 Teleop started - R2=throttle, L2=brake, Stick=steering')

    def _init_serial(self):
        try:
            self.serial_conn = serial.Serial(port=self.serial_port, baudrate=self.baud_rate, timeout=0.1)
            time.sleep(2.0)
            self.serial_conn.write(b"ARM\n")
            self.get_logger().info(f'Serial connected: {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial failed: {e}')
            self.serial_conn = None

    def _trigger_to_pct(self, raw_value: float) -> float:
        pct = (1.0 - raw_value) / 2.0
        if pct < 0.03:
            return 0.0
        return min(1.0, pct)

    def _joy_callback(self, msg: Joy):
        if len(msg.buttons) > self.BUTTON_CROSS:
            if self._button_pressed(msg.buttons, self.BUTTON_CROSS):
                self.enabled = not self.enabled
                self.get_logger().info(f'Enabled: {self.enabled}')
            if self._button_pressed(msg.buttons, self.BUTTON_CIRCLE):
                self.get_logger().warn('E-STOP!')
                self.enabled = False
                self._throttle = 90
                self._brake = 180
        self._prev_buttons = list(msg.buttons)

        if not self.enabled or len(msg.axes) < 5:
            return

        steering_raw = msg.axes[self.AXIS_LEFT_STICK_X]
        if abs(steering_raw) < 0.03:
            steering_raw = 0.0
        throttle_pct = self._trigger_to_pct(msg.axes[self.AXIS_R2])
        brake_pct = self._trigger_to_pct(msg.axes[self.AXIS_L2])

        self._steer = int(90 + steering_raw * 90)
        self._throttle = int(90 + throttle_pct * 90)
        self._brake = int(brake_pct * 180)

    def _send_callback(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            cmd = f"S{self._steer},T{self._throttle},B{self._brake}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
        except SERIAL_ERRORS:
            self.serial_conn = None

    def _button_pressed(self, buttons: list, button_id: int) -> bool:
        if button_id >= len(buttons):
            return False
        current = buttons[button_id] == 1
        previous = self._prev_buttons[button_id] == 1 if button_id < len(self._prev_buttons) else False
        return current and not previous

def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.write(b"DISARM\n")
            node.serial_conn.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
