#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import termios
import time

SERIAL_ERRORS = (serial.SerialException, OSError, termios.error)

class PS4TeleopNode(Node):

    # declare buttons/axis refeerence
    AXIS_LEFT_STICK_X = 0
    AXIS_L2 = 3
    AXIS_R2 = 4
    BUTTON_CROSS = 0
    BUTTON_CIRCLE = 1

    # init all parameters and start subscribers and and publishers
    def __init__(self):
        super().__init__('ps4_teleop_simple')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.enabled = True
        self._prev_buttons = []

        self.steer_center = 90
        self.steer_range = 90       # +/- from center (90 = full 0-180)

        self.throttle_min = 90      # Neutral
        self.throttle_max = 150     # Full throttle

        self.brake_min = 0          # No brake
        self.brake_max = 150        # Full brake

        self._steer = self.steer_center
        self._throttle = self.throttle_min
        self._brake = self.brake_min

        self.serial_conn = None
        self._init_serial()
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.create_timer(0.05, self._send_callback)
        self.get_logger().info('PS4 Teleop started - R2=throttle, L2=brake, Stick=steering')

    # init serial connction
    def _init_serial(self):
        try:
            self.serial_conn = serial.Serial(port=self.serial_port, baudrate=self.baud_rate, timeout=0.1)
            time.sleep(2.0)
            self.serial_conn.write(b"ARM\n")
            self.get_logger().info(f'Serial connected: {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial failed: {e}')
            self.serial_conn = None

    # convert ps4 trigger output. 0 = not pressed, 1 = full preess (yo ir alos has a deadzone)
    def _trigger_to_pct(self, raw_value: float) -> float:
        pct = (1.0 - raw_value) / 2.0
        if pct < 0.03:
            return 0.0
        return min(1.0, pct)

    def _joy_callback(self, msg: Joy):
        if len(msg.buttons) > self.BUTTON_CROSS:
            # if cross, preessd toggle enableor not
            if self._button_pressed(msg.buttons, self.BUTTON_CROSS):
                self.enabled = not self.enabled
                self.get_logger().info(f'Enabled: {self.enabled}')
            # if circle pressed, emergency stop
            if self._button_pressed(msg.buttons, self.BUTTON_CIRCLE):
                self.get_logger().warn('E-STOP!')
                self.enabled = False
                self._throttle = self.throttle_min
                self._brake = self.brake_max
        self._prev_buttons = list(msg.buttons)

        if not self.enabled or len(msg.axes) < 5:
            return
        
        # apply the target values

        steering_raw = msg.axes[self.AXIS_LEFT_STICK_X]
        if abs(steering_raw) < 0.03:
            steering_raw = 0.0
        throttle_pct = self._trigger_to_pct(msg.axes[self.AXIS_R2])
        brake_pct = self._trigger_to_pct(msg.axes[self.AXIS_L2])

        self._steer = int(self.steer_center + steering_raw * self.steer_range)
        self._throttle = int(self.throttle_min + throttle_pct * (self.throttle_max - self.throttle_min))
        self._brake = int(self.brake_min + brake_pct * (self.brake_max - self.brake_min))

    # seend command to arduino
    def _send_callback(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            cmd = f"S{self._steer},T{self._throttle},B{self._brake}\n" # in the form T90, etc
            self.serial_conn.write(cmd.encode('utf-8'))
        except SERIAL_ERRORS:
            self.serial_conn = None

    # rturn if ther button press is digital_press
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
