#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

try:
    import odrive
    from odrive.enums import AxisState, ControlMode, InputMode
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False

LSS_THROTTLE_ID = 1
LSS_BRAKE_ID = 2
LSS_GEAR_ID = 3

LSS_THROTTLE_NEUTRAL = -900
LSS_THROTTLE_MAX = -150
LSS_BRAKE_RELEASED = 900
LSS_BRAKE_ENGAGED = -100
LSS_GEAR_REVERSE = -330
LSS_GEAR_NEUTRAL = -100
LSS_GEAR_HIGH = 200


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

    GEAR_LSS_POSITIONS = {0: LSS_GEAR_REVERSE, 1: LSS_GEAR_NEUTRAL, 2: LSS_GEAR_HIGH}

    def __init__(self):
        super().__init__('ackermann_drive_node')

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('hardware_mode', False)
        self.declare_parameter('serial_port', '/dev/lss_controller')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('use_odrive', True)

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

        self.serial_conn = None
        self._lss_connected = False
        self._last_throttle_lss = None
        self._last_brake_lss = None
        self._last_gear_lss = None

        self._odrive = None
        self._odrive_axis = None
        self._odrive_anchor = 0.0
        self._odrive_connected = False
        self._odrive_last_attempt = 0.0
        self._odrive_retry_interval = 5.0

        if self.hardware_mode:
            self._init_lss()
            use_odrive = self.get_parameter('use_odrive').value
            if use_odrive:
                self._init_odrive()

        self.last_time = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            qos
        )

        self.throttle_pub = self.create_publisher(
            Float64MultiArray,
            '/throttle_controller/commands',
            qos
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos
        )

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.control_timer = self.create_timer(0.02, self.control_loop)

        mode_str = 'HARDWARE' if self.hardware_mode else 'SIMULATION'
        self.get_logger().info(f'Ackermann drive node started ({mode_str})')

    def _init_lss(self):
        try:
            import serial
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('serial_baud').value
            self.serial_conn = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            time.sleep(0.5)
            self.serial_conn.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f'LSS serial init failed: {e}')
            self.serial_conn = None
            return

        detected = []
        for servo_id in [LSS_THROTTLE_ID, LSS_BRAKE_ID, LSS_GEAR_ID]:
            if self._query_lss_servo(servo_id):
                detected.append(servo_id)
                self.get_logger().info(f'LSS servo {servo_id} detected')
            else:
                self.get_logger().warn(f'LSS servo {servo_id} not responding')

        if LSS_BRAKE_ID not in detected:
            self.get_logger().error('Brake servo not detected — LSS control disabled')
            self._lss_connected = False
            return

        self._send_lss(f'#{LSS_THROTTLE_ID}D{LSS_THROTTLE_NEUTRAL}\r')
        self._send_lss(f'#{LSS_BRAKE_ID}D{LSS_BRAKE_RELEASED}\r')
        self._send_lss(f'#{LSS_GEAR_ID}D{LSS_GEAR_NEUTRAL}\r')

        self._last_throttle_lss = LSS_THROTTLE_NEUTRAL
        self._last_brake_lss = LSS_BRAKE_RELEASED
        self._last_gear_lss = LSS_GEAR_NEUTRAL

        self._lss_connected = True
        self.get_logger().info(f'LSS bus ready — {len(detected)}/3 servos detected')

    def _query_lss_servo(self, servo_id):
        if self.serial_conn is None:
            return False
        try:
            self.serial_conn.reset_input_buffer()
            cmd = f'#{servo_id}QS\r'
            self.serial_conn.write(cmd.encode('utf-8'))
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
            self.serial_conn.write(cmd.encode('utf-8'))
            self.serial_conn.flush()
        except Exception as e:
            self.get_logger().warn(f'LSS write error: {e}')
            self._lss_connected = False

    def _init_odrive(self):
        if not ODRIVE_AVAILABLE:
            self.get_logger().warn('ODrive library not installed, steering will not be controlled')
            return
        self._connect_odrive()

    def _connect_odrive(self):
        self._odrive_last_attempt = time.monotonic()
        try:
            self.get_logger().info('Searching for ODrive...')
            self._odrive = odrive.find_any(timeout=10)
            if self._odrive is None:
                self.get_logger().error('ODrive not found')
                return
            self._odrive_axis = self._odrive.axis0
            self._odrive_axis.config.motor.current_soft_max = 20.0
            self._odrive_axis.config.motor.current_hard_max = 30.0
            ctrl = self._odrive_axis.controller
            ctrl.config.control_mode = ControlMode.POSITION_CONTROL
            ctrl.config.input_mode = InputMode.PASSTHROUGH
            ctrl.config.vel_limit = 15.0
            if self._odrive_axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
                self._odrive_axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
                deadline = time.monotonic() + 2.0
                while self._odrive_axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
                    if time.monotonic() > deadline:
                        self.get_logger().error('ODrive closed-loop timeout')
                        return
                    time.sleep(0.05)
            self._odrive_anchor = self._odrive_axis.pos_estimate
            self._odrive_connected = True
            self.get_logger().info(f'ODrive connected, anchor={self._odrive_anchor:.3f}')
        except Exception as e:
            self.get_logger().error(f'ODrive connection failed: {e}')
            self._odrive_connected = False

    def _command_odrive_steering(self, steering_angle):
        if not self._odrive_connected:
            now = time.monotonic()
            if now - self._odrive_last_attempt > self._odrive_retry_interval:
                self._connect_odrive()
            return
        try:
            max_pos = 6.0
            normalized = steering_angle / self.MAX_STEERING_ANGLE
            target_turns = normalized * max_pos
            absolute_pos = self._odrive_anchor + target_turns
            self._odrive_axis.controller.input_pos = absolute_pos
        except Exception as e:
            self.get_logger().error(f'ODrive command failed: {e}')
            self._odrive_connected = False

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

    def _command_gear(self, gear):
        lss_pos = self.GEAR_LSS_POSITIONS[gear]
        if lss_pos != self._last_gear_lss:
            self._send_lss(f'#{LSS_GEAR_ID}D{lss_pos}\r')
            self._last_gear_lss = lss_pos
        self._current_gear = gear

    def cmd_vel_callback(self, msg: Twist):
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

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0 or dt > 1.0:
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

        if self.hardware_mode and self._lss_connected:
            if ramped_speed > 0.01:
                throttle_deg = int(abs(ramped_speed) / self.MAX_SPEED * 75)
                throttle_deg = max(0, min(75, throttle_deg))
                brake_deg = 180
            elif ramped_speed < -0.01:
                throttle_deg = int(abs(ramped_speed) / self.MAX_SPEED * 75)
                throttle_deg = max(0, min(75, throttle_deg))
                brake_deg = 180
            else:
                throttle_deg = 0
                brake_deg = 180

            throttle_lss = (throttle_deg - 90) * 10
            brake_lss = (brake_deg - 90) * 10

            if throttle_lss != self._last_throttle_lss:
                self._send_lss(f'#{LSS_THROTTLE_ID}D{throttle_lss}\r')
                self._last_throttle_lss = throttle_lss

            if brake_lss != self._last_brake_lss:
                self._send_lss(f'#{LSS_BRAKE_ID}D{brake_lss}\r')
                self._last_brake_lss = brake_lss

            self._process_gear(ramped_speed)

            self._command_odrive_steering(steering_angle)

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
        if self.hardware_mode:
            if self._odrive_connected and self._odrive_axis is not None:
                try:
                    self._odrive_axis.controller.input_pos = self._odrive_axis.pos_estimate
                    self._odrive_axis.requested_state = AxisState.IDLE
                except Exception:
                    pass
            if self.serial_conn is not None:
                try:
                    self._send_lss(f'#{LSS_THROTTLE_ID}D{LSS_THROTTLE_NEUTRAL}\r')
                    self._send_lss(f'#{LSS_BRAKE_ID}D{LSS_BRAKE_ENGAGED}\r')
                    time.sleep(0.05)
                    self._send_lss(f'#{LSS_BRAKE_ID}H\r')
                    self._send_lss(f'#{LSS_THROTTLE_ID}L\r')
                    self._send_lss(f'#{LSS_GEAR_ID}L\r')
                    time.sleep(0.05)
                    self.serial_conn.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
