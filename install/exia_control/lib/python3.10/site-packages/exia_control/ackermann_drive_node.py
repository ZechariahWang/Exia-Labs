#!/usr/bin/env python3
import math
import time
import signal
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

try:
    from Phidget22.Devices.MotorPositionController import MotorPositionController
    from Phidget22.PhidgetException import PhidgetException
    PHIDGETS_AVAILABLE = True
except ImportError:
    PHIDGETS_AVAILABLE = False

try:
    import Jetson.GPIO as GPIO
    JETSON_GPIO_AVAILABLE = True
except Exception:
    JETSON_GPIO_AVAILABLE = False

LSS_THROTTLE_ID                      = 1
LSS_BRAKE_ID                         = 2

LSS_THROTTLE_NEUTRAL                 = 5
LSS_THROTTLE_MAX                     = 550
LSS_BRAKE_RELEASED                   = 0
LSS_BRAKE_ENGAGED                    = -100

LSS_SEND_DEADBAND                    = 8
LSS_SEND_MIN_INTERVAL                = 0.025
LSS_SERVO_SPEED                      = 600

GEAR_PWM_PIN                         = 32
GEAR_PWM_FREQ                        = 50
GEAR_PWM_REVERSE                     = 5.67
GEAR_PWM_NEUTRAL                     = 6.94
GEAR_PWM_HIGH                        = 8.61
GEAR_PWM                             = {0: GEAR_PWM_REVERSE, 1: GEAR_PWM_NEUTRAL, 2: GEAR_PWM_HIGH}

DEFAULT_PUBLISH_TF                   = True
DEFAULT_HARDWARE_MODE                = False
DEFAULT_SERIAL_PORT                  = '/dev/lss_controller'
DEFAULT_SERIAL_BAUD                  = 115200
DEFAULT_USE_PHIDGETS                 = True
DEFAULT_PHIDGETS_HUB_PORT            = 0
DEFAULT_MOTOR_DEGREES_AT_MAX_STEER   = 2160.0
DEFAULT_STEERING_KP                  = 400.0
DEFAULT_STEERING_KI                  = 0.0
DEFAULT_STEERING_KD                  = 150.0
DEFAULT_STEERING_VELOCITY_LIMIT      = 10000.0
DEFAULT_STEERING_ACCELERATION        = 50000.0
DEFAULT_STEERING_DEAD_BAND           = 2.0
DEFAULT_STEERING_CURRENT_LIMIT       = 3.0


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _lerp(a, b, t):
    return a + (b - a) * t


class AckermannDriveNode(Node):
    WHEELBASE = 1.3
    TRACK_WIDTH = 1.1
    WHEEL_RADIUS = 0.3
    MAX_STEERING_ANGLE = 0.6
    MAX_SPEED = 5.0
    MAX_ACCEL = 2.0
    MAX_DECEL = 4.0

    GEAR_REVERSE = 0
    GEAR_NEUTRAL = 1
    GEAR_HIGH = 2
    GEAR_SHIFT_DELAY = 0.3

    def __init__(self):
        super().__init__('ackermann_drive_node')

        self.declare_parameter('publish_tf', DEFAULT_PUBLISH_TF)
        self.declare_parameter('hardware_mode', DEFAULT_HARDWARE_MODE)
        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        self.declare_parameter('serial_baud', DEFAULT_SERIAL_BAUD)
        self.declare_parameter('use_phidgets', DEFAULT_USE_PHIDGETS)
        self.declare_parameter('phidgets_hub_port', DEFAULT_PHIDGETS_HUB_PORT)
        self.declare_parameter('motor_degrees_at_max_steer', DEFAULT_MOTOR_DEGREES_AT_MAX_STEER)
        self.declare_parameter('steering_kp', DEFAULT_STEERING_KP)
        self.declare_parameter('steering_ki', DEFAULT_STEERING_KI)
        self.declare_parameter('steering_kd', DEFAULT_STEERING_KD)
        self.declare_parameter('steering_velocity_limit', DEFAULT_STEERING_VELOCITY_LIMIT)
        self.declare_parameter('steering_acceleration', DEFAULT_STEERING_ACCELERATION)
        self.declare_parameter('steering_dead_band', DEFAULT_STEERING_DEAD_BAND)
        self.declare_parameter('steering_current_limit', DEFAULT_STEERING_CURRENT_LIMIT)

        self.publish_tf = self.get_parameter('publish_tf').value
        self.hardware_mode = self.get_parameter('hardware_mode').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.current_speed = 0.0
        self.last_cmd_time = None
        self.cmd_timeout = 0.5

        self._current_gear = self.GEAR_NEUTRAL
        self._target_gear = self.GEAR_NEUTRAL
        self._gear_shift_time = 0.0
        self._gear_pwm = None
        self._gear_ok = False

        self.serial_conn = None
        self._lss_ok = False
        self._lss_pending = None
        self._lss_open_time = 0.0
        self._last_sent_throttle = None
        self._last_sent_brake = None
        self._last_send_time_throttle = 0.0
        self._last_send_time_brake = 0.0

        self._phidgets_controller = None
        self._motor_ok = False
        self._motor_lock = threading.Lock()
        self._phidgets_runaway_count = 0
        self._motor_degrees_at_max_steer = self.get_parameter('motor_degrees_at_max_steer').value

        self._estop = False

        if self.hardware_mode:
            self._init_lss()
            self._init_gear_servo()
            if self.get_parameter('use_phidgets').value:
                self._init_phidgets()

        self.last_time = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', qos)
        self.throttle_pub = self.create_publisher(
            Float64MultiArray, '/throttle_controller/commands', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self.control_timer = self.create_timer(0.02, self.control_loop)
        if self.hardware_mode:
            self.create_timer(2.0, self._reconnect_check)

        mode_str = 'HARDWARE' if self.hardware_mode else 'SIMULATION'
        self.get_logger().info(f'Ackermann drive node started ({mode_str})')

    def _init_lss(self):
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        self._lss_ok = False
        self._lss_pending = None
        try:
            import serial
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('serial_baud').value
            self._lss_pending = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self._lss_open_time = time.monotonic()
        except Exception:
            self._lss_pending = None

    def _init_lss_finish(self):
        conn = self._lss_pending
        self._lss_pending = None
        if conn is None:
            return
        try:
            conn.reset_input_buffer()
        except Exception:
            try:
                conn.close()
            except Exception:
                pass
            return
        self.serial_conn = conn

        servo_ok = {}
        for sid, name in [(LSS_THROTTLE_ID, 'throttle'), (LSS_BRAKE_ID, 'brake')]:
            if self._query_servo(sid):
                self.get_logger().info(f'LSS servo {sid} ({name}): OK')
                self._send_lss(f'#{sid}SD{LSS_SERVO_SPEED}\r')
                servo_ok[sid] = True
            else:
                self.get_logger().warn(f'LSS servo {sid} ({name}): no response')
                servo_ok[sid] = False

        if not servo_ok.get(LSS_THROTTLE_ID):
            self.get_logger().error('Throttle servo not responding — LSS disabled')
            return

        self._lss_ok = True
        self._send_lss(f'#{LSS_THROTTLE_ID}D{LSS_THROTTLE_NEUTRAL}\r')
        self._send_lss(f'#{LSS_BRAKE_ID}D{LSS_BRAKE_RELEASED}\r')
        self._last_sent_throttle = None
        self._last_sent_brake = None

    def _query_servo(self, servo_id):
        if self.serial_conn is None:
            return False
        try:
            self.serial_conn.reset_input_buffer()
            self.serial_conn.write(f'#{servo_id}QS\r'.encode())
            self.serial_conn.flush()
            time.sleep(0.05)
            response = b''
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                if self.serial_conn.in_waiting > 0:
                    response += self.serial_conn.read(self.serial_conn.in_waiting)
                    if b'\r' in response:
                        break
                time.sleep(0.01)
            return f'*{servo_id}'.encode() in response
        except Exception:
            return False

    def _send_lss(self, cmd):
        if self.serial_conn is None:
            return
        try:
            self.serial_conn.write(cmd.encode())
            self.serial_conn.flush()
        except Exception:
            self._lss_ok = False
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

    def _init_gear_servo(self):
        if not JETSON_GPIO_AVAILABLE:
            return
        if self._gear_ok:
            return
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(GEAR_PWM_PIN, GPIO.OUT)
            self._gear_pwm = GPIO.PWM(GEAR_PWM_PIN, GEAR_PWM_FREQ)
            self._gear_pwm.start(GEAR_PWM[self.GEAR_NEUTRAL])
            self._gear_ok = True
            self.get_logger().info(f'Gear servo on pin {GEAR_PWM_PIN}: OK')
        except Exception as e:
            self.get_logger().warn(f'Gear servo init failed: {e}')

    def _init_phidgets(self):
        if not PHIDGETS_AVAILABLE:
            self.get_logger().warn('Phidget22 not installed — steering unavailable')
            return
        try:
            ch = MotorPositionController()
            ch.setHubPort(self.get_parameter('phidgets_hub_port').value)
            ch.setIsHubPortDevice(False)
            ch.setOnAttachHandler(self._on_attach)
            ch.setOnDetachHandler(self._on_detach)
            self._phidgets_controller = ch
            ch.open()
            self.get_logger().info('MotorPositionController channel opened, waiting for attach...')
        except PhidgetException as e:
            self.get_logger().warn(f'Phidgets init failed: {e}')

    def _on_attach(self, sender):
        try:
            sender.setRescaleFactor(360.0 / (300 * 4 * 4.25))
            sender.setCurrentLimit(self.get_parameter('steering_current_limit').value)
            sender.setVelocityLimit(self.get_parameter('steering_velocity_limit').value)
            sender.setAcceleration(self.get_parameter('steering_acceleration').value)
            sender.setDeadBand(self.get_parameter('steering_dead_band').value)
            sender.setKp(self.get_parameter('steering_kp').value)
            sender.setKi(self.get_parameter('steering_ki').value)
            sender.setKd(self.get_parameter('steering_kd').value)
            sender.addPositionOffset(-sender.getPosition())
            sender.setEngaged(True)
            try:
                sender.enableFailsafe(500)
            except Exception:
                pass
            with self._motor_lock:
                self._motor_ok = True
                self._phidgets_runaway_count = 0
            self.get_logger().info('MotorPositionController attached and engaged')
        except PhidgetException as e:
            self.get_logger().error(f'MotorPositionController config failed: {e}')

    def _on_detach(self, sender):
        with self._motor_lock:
            self._motor_ok = False
            self._phidgets_controller = None
        try:
            sender.close()
        except Exception:
            pass
        self.get_logger().warn('MotorPositionController detached')

    def _command_steering(self, steering_angle):
        with self._motor_lock:
            if not self._motor_ok or self._phidgets_controller is None:
                return
            ctrl = self._phidgets_controller
        try:
            normalized = steering_angle / self.MAX_STEERING_ANGLE
            target_pos = normalized * self._motor_degrees_at_max_steer
            ctrl.setTargetPosition(target_pos)
            ctrl.resetFailsafe()
            duty = abs(ctrl.getDutyCycle())
            if duty > 0.95:
                self._phidgets_runaway_count += 1
                if self._phidgets_runaway_count > 25:
                    self.get_logger().error('Steering motor runaway — disengaging')
                    ctrl.setEngaged(False)
                    with self._motor_lock:
                        self._motor_ok = False
            else:
                self._phidgets_runaway_count = 0
        except Exception:
            pass

    def _command_gear(self, gear):
        if self._gear_pwm is not None and self._gear_ok:
            duty = GEAR_PWM.get(gear)
            if duty is not None:
                self._gear_pwm.ChangeDutyCycle(duty)
        self._current_gear = gear

    def _reconnect_check(self):
        if not self._lss_ok and self._lss_pending is None:
            self._init_lss()

        if not self._gear_ok:
            self._init_gear_servo()

        with self._motor_lock:
            motor_ok = self._motor_ok
        if not motor_ok and self._phidgets_controller is None:
            if self.get_parameter('use_phidgets').value:
                self._init_phidgets()

    def _process_gear(self, ramped_speed):
        if ramped_speed > 0.05:
            self._target_gear = self.GEAR_HIGH
        elif ramped_speed < -0.05:
            self._target_gear = self.GEAR_REVERSE
        else:
            self._target_gear = self.GEAR_NEUTRAL

        if self._target_gear == self._current_gear:
            return

        now = time.monotonic()
        if now - self._gear_shift_time < self.GEAR_SHIFT_DELAY:
            return

        if self._current_gear == self.GEAR_HIGH and self._target_gear == self.GEAR_REVERSE:
            self._command_gear(self.GEAR_NEUTRAL)
            self._gear_shift_time = now
            return

        if self._current_gear == self.GEAR_REVERSE and self._target_gear == self.GEAR_HIGH:
            self._command_gear(self.GEAR_NEUTRAL)
            self._gear_shift_time = now
            return

        self._command_gear(self._target_gear)
        self._gear_shift_time = now

    def cmd_vel_callback(self, msg: Twist):
        if self._estop:
            return
        self.linear_vel = max(-self.MAX_SPEED, min(self.MAX_SPEED, msg.linear.x))
        self.angular_vel = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _apply_acceleration_limit(self, target_speed, dt):
        speed_diff = target_speed - self.current_speed
        if speed_diff > 0:
            max_change = self.MAX_ACCEL * dt
        else:
            max_change = self.MAX_DECEL * dt
        if abs(speed_diff) > max_change:
            self.current_speed += max_change if speed_diff > 0 else -max_change
        else:
            self.current_speed = target_speed
        return self.current_speed

    def _safe_stop(self):
        self._estop = True
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.current_speed = 0.0

        if self.hardware_mode and self._lss_ok:
            self._send_lss(f'#{LSS_THROTTLE_ID}D{LSS_THROTTLE_NEUTRAL}\r')
            self._send_lss(f'#{LSS_BRAKE_ID}D{LSS_BRAKE_ENGAGED}\r')
            self._last_sent_throttle = None
            self._last_sent_brake = None

        if self.hardware_mode:
            self._command_steering(0.0)
            self._command_gear(self.GEAR_NEUTRAL)
            self._gear_shift_time = time.monotonic()

        self._current_gear = self.GEAR_NEUTRAL
        self._target_gear = self.GEAR_NEUTRAL

    def _signal_handler(self, _sig, _frame):
        self._safe_stop()
        raise SystemExit(0)

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0 or dt > 1.0:
            return

        if self._lss_pending is not None and time.monotonic() - self._lss_open_time >= 0.5:
            self._init_lss_finish()

        if self._estop:
            if self.hardware_mode and self._lss_ok:
                self._send_lss(f'#{LSS_THROTTLE_ID}D{LSS_THROTTLE_NEUTRAL}\r')
                self._send_lss(f'#{LSS_BRAKE_ID}D{LSS_BRAKE_ENGAGED}\r')
            if self.hardware_mode:
                self._command_steering(0.0)
            self.update_odometry(dt, 0.0, 0.0)
            return

        if self.last_cmd_time is not None:
            cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9
            if cmd_age > self.cmd_timeout:
                self.linear_vel = 0.0
                self.angular_vel = 0.0

        ramped_speed = self._apply_acceleration_limit(self.linear_vel, dt)

        if abs(ramped_speed) < 0.01:
            steering_angle = 0.0
            wheel_speed = 0.0
        else:
            if abs(self.angular_vel) > 0.001:
                turn_radius = ramped_speed / self.angular_vel
                steering_angle = math.atan(self.WHEELBASE / turn_radius)
            else:
                steering_angle = 0.0

            steering_angle = max(-self.MAX_STEERING_ANGLE,
                               min(self.MAX_STEERING_ANGLE, steering_angle))

            wheel_speed = ramped_speed / self.WHEEL_RADIUS

        if self.hardware_mode and self._lss_ok:
            now = time.monotonic()
            throttle_frac = _clamp(abs(ramped_speed) / self.MAX_SPEED, 0.0, 1.0)
            thr_int = int(round(_lerp(LSS_THROTTLE_NEUTRAL, LSS_THROTTLE_MAX, throttle_frac)))

            if abs(ramped_speed) > 0.01:
                brk_int = LSS_BRAKE_RELEASED
            else:
                brk_int = LSS_BRAKE_ENGAGED

            thr_at_neutral = (thr_int == LSS_THROTTLE_NEUTRAL)
            thr_needs_send = (
                self._last_sent_throttle is None
                or abs(thr_int - self._last_sent_throttle) >= LSS_SEND_DEADBAND
                or (thr_at_neutral and self._last_sent_throttle != LSS_THROTTLE_NEUTRAL))
            if thr_needs_send and (now - self._last_send_time_throttle) >= LSS_SEND_MIN_INTERVAL:
                self._send_lss(f'#{LSS_THROTTLE_ID}D{thr_int}T50\r')
                self._last_sent_throttle = thr_int
                self._last_send_time_throttle = now

            brk_needs_send = (
                self._last_sent_brake is None
                or self._last_sent_brake != brk_int)
            if brk_needs_send and (now - self._last_send_time_brake) >= LSS_SEND_MIN_INTERVAL:
                self._send_lss(f'#{LSS_BRAKE_ID}D{brk_int}T50\r')
                self._last_sent_brake = brk_int
                self._last_send_time_brake = now

            self._process_gear(ramped_speed)

        if self.hardware_mode:
            self._command_steering(steering_angle)

        left_steer, right_steer = self.compute_ackermann_angles(steering_angle)

        steering_msg = Float64MultiArray()
        steering_msg.data = [left_steer, right_steer]
        self.steering_pub.publish(steering_msg)

        throttle_msg = Float64MultiArray()
        throttle_msg.data = [wheel_speed, wheel_speed]
        self.throttle_pub.publish(throttle_msg)

        self.update_odometry(dt, steering_angle, ramped_speed)

    def compute_ackermann_angles(self, center_angle):
        if abs(center_angle) < 0.001:
            return 0.0, 0.0

        turn_radius = self.WHEELBASE / math.tan(abs(center_angle))

        inner_radius = turn_radius - self.TRACK_WIDTH / 2
        outer_radius = turn_radius + self.TRACK_WIDTH / 2

        inner_angle = math.atan(self.WHEELBASE / inner_radius)
        outer_angle = math.atan(self.WHEELBASE / outer_radius)

        if center_angle > 0:
            return inner_angle, outer_angle
        else:
            return -outer_angle, -inner_angle

    def update_odometry(self, dt, steering_angle, speed):
        old_theta = self.theta

        if abs(steering_angle) < 0.001:
            self.x += speed * math.cos(self.theta) * dt
            self.y += speed * math.sin(self.theta) * dt
        else:
            turn_radius = self.WHEELBASE / math.tan(steering_angle)
            angular_vel = speed / turn_radius
            self.theta += angular_vel * dt
            mid_theta = (old_theta + self.theta) / 2.0
            self.x += speed * math.cos(mid_theta) * dt
            self.y += speed * math.sin(mid_theta) * dt

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        odom_msg.twist.twist.linear.x = speed
        odom_msg.twist.twist.angular.z = self.angular_vel if abs(steering_angle) > 0.001 else 0.0

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._safe_stop()

        if self.hardware_mode:
            if self._lss_pending is not None:
                try:
                    self._lss_pending.close()
                except Exception:
                    pass
                self._lss_pending = None

            if self.serial_conn is not None:
                try:
                    self._send_lss(f'#{LSS_BRAKE_ID}H\r')
                    self._send_lss(f'#{LSS_THROTTLE_ID}L\r')
                    time.sleep(0.05)
                    self.serial_conn.close()
                except Exception:
                    pass

            with self._motor_lock:
                ctrl = self._phidgets_controller
                if ctrl is not None:
                    try:
                        if self._motor_ok:
                            ctrl.setTargetPosition(0.0)
                            ctrl.setEngaged(False)
                        self._motor_ok = False
                        self._phidgets_controller = None
                        ctrl.close()
                    except Exception:
                        pass

            if self._gear_pwm is not None:
                try:
                    self._gear_pwm.ChangeDutyCycle(GEAR_PWM_NEUTRAL)
                    time.sleep(0.1)
                    self._gear_pwm.stop()
                    GPIO.cleanup()
                except Exception:
                    pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
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
