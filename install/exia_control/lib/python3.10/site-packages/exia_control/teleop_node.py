#!/usr/bin/env python3
import sys
import math
import time
import signal
import atexit
import threading
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


SPEED_STEP = 0.1
STEER_STEP = 0.05
MAX_SPEED = 2.0
MAX_REVERSE = 0.5
MAX_STEER = 0.4
WHEELBASE = 1.3
JOY_DEADZONE = 0.08
JOY_TIMEOUT = 0.3

GEAR_REVERSE = 0
GEAR_NEUTRAL = 1
GEAR_HIGH = 2
GEAR_NAMES = {0: 'R', 1: 'N', 2: 'H'}


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        self.target_speed = 0.0
        self.target_steer = 0.0
        self.gear = GEAR_NEUTRAL
        self.running = True
        self._last_hud_time = 0.0

        self._joy_speed = 0.0
        self._joy_steer = 0.0
        self._joy_active = False
        self._joy_last_time = 0.0
        self._joy_src = ''

        self.old_settings = None
        self._setup_terminal()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self._cancel_navigation()
        self.timer = self.create_timer(0.05, self._publish)

        self.key_thread = threading.Thread(target=self._key_reader, daemon=True)
        self.key_thread.start()
        self._print_hud()

    def _cancel_navigation(self):
        client = self.create_client(Trigger, '/navigation/cancel')
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().warn('Cancelled active navigation')
        self.destroy_client(client)

    def _setup_terminal(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        atexit.register(self._restore_terminal)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _restore_terminal(self):
        if self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.old_settings = None

    def _signal_handler(self, _sig, _frame):
        self.running = False
        self._stop_robot()
        self._restore_terminal()
        raise SystemExit(0)

    def _stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def _joy_cb(self, msg):
        if len(msg.axes) < 4 or len(msg.buttons) < 4:
            return

        r2 = msg.axes[5] if len(msg.axes) > 5 else 1.0
        l2 = msg.axes[2] if len(msg.axes) > 2 else 1.0
        left_y = msg.axes[1]
        right_x = msg.axes[3]

        throttle = (1.0 - r2) / 2.0
        brake = (1.0 - l2) / 2.0
        steer = right_x

        if abs(left_y) > JOY_DEADZONE:
            if left_y > 0:
                throttle = max(throttle, left_y)
            else:
                brake = max(brake, -left_y)

        if throttle > JOY_DEADZONE:
            if self.gear == GEAR_HIGH:
                self._joy_speed = throttle * MAX_SPEED
            elif self.gear == GEAR_REVERSE:
                self._joy_speed = -throttle * MAX_REVERSE
            else:
                self._joy_speed = 0.0
        elif brake > JOY_DEADZONE:
            if self._joy_speed > 0:
                self._joy_speed = max(self._joy_speed - brake * MAX_SPEED, 0.0)
            elif self._joy_speed < 0:
                self._joy_speed = min(self._joy_speed + brake * MAX_REVERSE, 0.0)
            else:
                self._joy_speed = 0.0
        else:
            self._joy_speed = 0.0

        if abs(steer) > JOY_DEADZONE:
            self._joy_steer = steer * MAX_STEER
        else:
            self._joy_steer = 0.0

        has_input = throttle > JOY_DEADZONE or brake > JOY_DEADZONE or abs(steer) > JOY_DEADZONE
        if has_input:
            self._joy_active = True
            self._joy_last_time = time.monotonic()

        if msg.buttons[0]:
            self.gear = GEAR_REVERSE
        elif msg.buttons[2]:
            self.gear = GEAR_HIGH
        elif msg.buttons[3]:
            self.gear = GEAR_NEUTRAL

        if msg.buttons[1]:
            self._joy_speed = 0.0
            self._joy_steer = 0.0
            self._joy_active = False
            self.target_speed = 0.0
            self.target_steer = 0.0
            self.gear = GEAR_NEUTRAL
            self._stop_robot()

        self._print_hud()

    def _key_reader(self):
        while self.running:
            try:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    ch = sys.stdin.read(1)

                    if ch == '\x1b':
                        if select.select([sys.stdin], [], [], 0.01)[0]:
                            sys.stdin.read(1)
                            if select.select([sys.stdin], [], [], 0.01)[0]:
                                sys.stdin.read(1)
                        continue

                    lower = ch.lower()

                    if lower == 'w':
                        if self.target_speed < -0.01:
                            self.target_speed = min(self.target_speed + SPEED_STEP, 0.0)
                        elif self.gear == GEAR_HIGH:
                            self.target_speed = min(self.target_speed + SPEED_STEP, MAX_SPEED)
                    elif lower == 's':
                        if self.target_speed > 0.01:
                            self.target_speed = max(self.target_speed - SPEED_STEP, 0.0)
                        elif self.gear == GEAR_REVERSE:
                            self.target_speed = max(self.target_speed - SPEED_STEP, -MAX_REVERSE)
                    elif lower == 'a':
                        self.target_steer = min(self.target_steer + STEER_STEP, MAX_STEER)
                    elif lower == 'd':
                        self.target_steer = max(self.target_steer - STEER_STEP, -MAX_STEER)
                    elif lower == 'x':
                        self.target_speed = 0.0
                        self.target_steer = 0.0
                    elif ch == ' ':
                        self.target_speed = 0.0
                        self.target_steer = 0.0
                        self.gear = GEAR_NEUTRAL
                        self._stop_robot()
                    elif lower == 'q':
                        if self.gear < GEAR_HIGH and abs(self.target_speed) < 0.1:
                            self.gear += 1
                    elif lower == 'e':
                        if self.gear > GEAR_REVERSE and abs(self.target_speed) < 0.1:
                            self.gear -= 1

                    self._print_hud()
            except Exception:
                if not self.running:
                    break

    def _publish(self):
        joy_active = self._joy_active or (time.monotonic() - self._joy_last_time) < JOY_TIMEOUT

        if joy_active:
            speed = self._joy_speed
            steer = self._joy_steer
            self._joy_src = ' [JOY]'
        else:
            speed = self.target_speed
            steer = self.target_steer
            self._joy_src = ''

        msg = Twist()
        msg.linear.x = speed

        if abs(speed) > 0.05 and abs(steer) > 0.001:
            turn_radius = WHEELBASE / math.tan(steer)
            msg.angular.z = speed / turn_radius
        else:
            msg.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)

    def _print_hud(self):
        now = time.monotonic()
        if now - self._last_hud_time < 0.05:
            return
        self._last_hud_time = now

        joy_active = self._joy_active or (now - self._joy_last_time) < JOY_TIMEOUT
        if joy_active:
            speed = self._joy_speed
            steer = self._joy_steer
        else:
            speed = self.target_speed
            steer = self.target_steer

        gear_name = GEAR_NAMES.get(self.gear, '?')
        sys.stdout.write('\r\033[K')
        sys.stdout.write(
            f'[TELEOP{self._joy_src}] Gear: {gear_name} | '
            f'Speed: {speed:+.1f} m/s | '
            f'Steer: {math.degrees(steer):+.0f} deg | '
            f'[W/S:speed A/D:steer Q/E:gear X:zero Space:stop]'
        )
        sys.stdout.flush()

    def destroy_node(self):
        self.running = False
        self._stop_robot()
        self._restore_terminal()
        sys.stdout.write('\n')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    print('Exia Teleop (Keyboard + Gamepad)')
    print('-------------------------------')
    print(f'  W/S     : speed +/- {SPEED_STEP} m/s')
    print(f'  A/D     : steer +/- {math.degrees(STEER_STEP):.0f} deg')
    print('  Q/E     : gear up / down (R-N-H)')
    print('  X       : zero speed + steering')
    print('  Space   : full stop + neutral')
    print('  Ctrl+C  : quit')
    print()
    print('  Gamepad: R2=throttle L2=brake RStick=steer')
    print('  X=reverse  Y=neutral  A(top)=high  B=e-stop')
    print()

    node = TeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
