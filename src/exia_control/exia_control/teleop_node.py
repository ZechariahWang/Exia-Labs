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
from std_srvs.srv import Trigger


SPEED_STEP = 0.1
STEER_STEP = 0.05
MAX_SPEED = 2.0
MAX_REVERSE = 0.5
MAX_STEER = 0.4
WHEELBASE = 1.3

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

        self.old_settings = None
        self._setup_terminal()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
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
        msg = Twist()
        msg.linear.x = self.target_speed

        if abs(self.target_speed) > 0.05 and abs(self.target_steer) > 0.001:
            turn_radius = WHEELBASE / math.tan(self.target_steer)
            msg.angular.z = self.target_speed / turn_radius
        else:
            msg.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)

    def _print_hud(self):
        now = time.monotonic()
        if now - self._last_hud_time < 0.05:
            return
        self._last_hud_time = now

        gear_name = GEAR_NAMES.get(self.gear, '?')
        sys.stdout.write('\r\033[K')
        sys.stdout.write(
            f'[TELEOP] Gear: {gear_name} | '
            f'Speed: {self.target_speed:+.1f} m/s | '
            f'Steer: {math.degrees(self.target_steer):+.0f} deg | '
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

    print('Exia WASD Teleop')
    print('----------------')
    print(f'  W/S     : speed +/- {SPEED_STEP} m/s')
    print(f'  A/D     : steer +/- {math.degrees(STEER_STEP):.0f} deg')
    print('  Q/E     : gear up / down (R-N-H)')
    print('  X       : zero speed + steering')
    print('  Space   : full stop + neutral')
    print('  Ctrl+C  : quit')
    print()
    print('  Velocity persists until changed.')
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
