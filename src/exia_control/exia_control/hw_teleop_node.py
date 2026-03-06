#!/usr/bin/env python3

import sys
import time
import signal
import threading
import subprocess
import os

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu, Joy
from std_msgs.msg import Int32, UInt32

# curl -fsSL  https://www.phidgets.com/downloads/setup_linux | sudo bash
# sudo apt install -y libphidget22 libphidget22-dev


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

try:
    import dearpygui.dearpygui as dpg
    DEARPYGUI_AVAILABLE = True
except ImportError:
    DEARPYGUI_AVAILABLE = False

try:
    import httpx
    BLUE_AVAILABLE = True
except ImportError:
    BLUE_AVAILABLE = False

# Need to assign manually using scripts
LSS_THROTTLE_ID                      = 1
LSS_BRAKE_ID                         = 0

LSS_THROTTLE_NEUTRAL                 = 0
LSS_THROTTLE_MAX                     = 330
LSS_BRAKE_RELEASED                   = -100
LSS_BRAKE_ENGAGED                    = -160

GEAR_NAMES                           = {0: 'R', 1: 'N', 2: 'H'}
GEAR_PWM_PIN                         = 32
GEAR_PWM_FREQ                        = 50
GEAR_PWM_REVERSE                     = 5.67
GEAR_PWM_NEUTRAL                     = 6.94
GEAR_PWM_HIGH                        = 8.61
GEAR_PWM                             = {0: GEAR_PWM_REVERSE, 1: GEAR_PWM_NEUTRAL, 2: GEAR_PWM_HIGH}

SPRING_RATE                          = 4.0
STEER_SPRING_RATE                    = 5.0

CMD_VEL_MAX_SPEED                    = 5.0
CMD_VEL_MAX_REVERSE                  = 0.5
CMD_VEL_MAX_STEER                    = 0.6
CMD_VEL_WHEELBASE                    = 1.3

LSS_SEND_DEADBAND                    = 8
LSS_SEND_MIN_INTERVAL                = 0.025
LSS_SERVO_SPEED                      = 600

DEFAULT_SERIAL_PORT                  = '/dev/lss_controller'
DEFAULT_SERIAL_BAUD                  = 115200
DEFAULT_PHIDGETS_HUB_PORT            = 3
DEFAULT_MOTOR_DEGREES_AT_MAX_STEER   = 10000.0
DEFAULT_STEERING_KP                  = 400.0
DEFAULT_STEERING_KI                  = 0.0
DEFAULT_STEERING_KD                  = 150.0
DEFAULT_STEERING_VELOCITY_LIMIT      = 10000.0
DEFAULT_STEERING_ACCELERATION        = 50000.0
DEFAULT_STEERING_DEAD_BAND           = 2.0
DEFAULT_STEERING_CURRENT_LIMIT       = 50
DEFAULT_REMOTE_MODE                  = False
DEFAULT_LAUNCH_SENSORS               = True
DEFAULT_GUI                          = False
DEFAULT_BLUE                         = False
DEFAULT_HEARTBEAT_RATE               = 10.0
DEFAULT_HEARTBEAT_TIMEOUT            = 1.5
DEFAULT_HEARTBEAT_ENABLED            = True

# clamp val
def _clamp(value, lo, hi):
    return max(lo, min(hi, value))

# linear interpolation 
def _lerp(a, b, t):
    return a + (b - a) * t

# exponential filter for jerky movements
class SmoothFilter:

    def __init__(self, default, tau):
        self._x = float(default)
        self._tau = float(tau)

    def update(self, target, dt):
        dt = _clamp(float(dt), 1e-4, 0.2)
        alpha = 1.0 - pow(2.0, -dt / self._tau) if self._tau > 1e-6 else 1.0
        self._x += alpha * (float(target) - self._x)
        return self._x

    def snap(self, value):
        self._x = float(value)

    @property
    def value(self):
        return self._x

class HwTeleopNode(Node):

    def __init__(self):
        super().__init__('hw_teleop_node')

        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        self.declare_parameter('serial_baud', DEFAULT_SERIAL_BAUD)
        self.declare_parameter('phidgets_hub_port', DEFAULT_PHIDGETS_HUB_PORT)
        self.declare_parameter('motor_degrees_at_max_steer', DEFAULT_MOTOR_DEGREES_AT_MAX_STEER)
        self.declare_parameter('steering_kp', DEFAULT_STEERING_KP)
        self.declare_parameter('steering_ki', DEFAULT_STEERING_KI)
        self.declare_parameter('steering_kd', DEFAULT_STEERING_KD)
        self.declare_parameter('steering_velocity_limit', DEFAULT_STEERING_VELOCITY_LIMIT)
        self.declare_parameter('steering_acceleration', DEFAULT_STEERING_ACCELERATION)
        self.declare_parameter('steering_dead_band', DEFAULT_STEERING_DEAD_BAND)
        self.declare_parameter('steering_current_limit', DEFAULT_STEERING_CURRENT_LIMIT)
        self.declare_parameter('remote_mode', DEFAULT_REMOTE_MODE)
        self.declare_parameter('launch_sensors', DEFAULT_LAUNCH_SENSORS)
        self.declare_parameter('gui', DEFAULT_GUI)
        self.declare_parameter('blue', DEFAULT_BLUE)
        self.declare_parameter('gear_pwm_pin', GEAR_PWM_PIN)
        self.declare_parameter('gear_pwm_reverse', GEAR_PWM_REVERSE)
        self.declare_parameter('gear_pwm_neutral', GEAR_PWM_NEUTRAL)
        self.declare_parameter('gear_pwm_high', GEAR_PWM_HIGH)
        self.declare_parameter('heartbeat_rate', DEFAULT_HEARTBEAT_RATE)
        self.declare_parameter('heartbeat_timeout', DEFAULT_HEARTBEAT_TIMEOUT)
        self.declare_parameter('heartbeat_enabled', DEFAULT_HEARTBEAT_ENABLED)

        self._remote_mode = self.get_parameter('remote_mode').value
        self._launch_sensors = self.get_parameter('launch_sensors').value

        self._gear_pwm_map = {
            0: self.get_parameter('gear_pwm_reverse').value,
            1: self.get_parameter('gear_pwm_neutral').value,
            2: self.get_parameter('gear_pwm_high').value,
        }

        self.serial_conn = None
        self._lss_ok = False
        self._lss_pending = None
        self._lss_open_time = 0.0
        self._gear_pwm = None
        self._gear_ok = False

        self._phidgets_controller = None
        self._motor_ok = False
        self._motor_lock = threading.Lock()
        self._motor_degrees_at_max_steer = self.get_parameter('motor_degrees_at_max_steer').value
        self._phidgets_runaway_count = 0

        self._throttle_ramp = 0.0
        self._brake_ramp = 0.0
        self._steer_ramp = 0.0

        self._throttle_f = SmoothFilter(LSS_THROTTLE_NEUTRAL, tau=0.04)
        self._brake_f = SmoothFilter(LSS_BRAKE_RELEASED, tau=0.03)

        self._gear = 1
        self._gear_lock = threading.Lock()
        self._last_gear_shift_time = 0.0
        self._last_tick_t = time.monotonic()

        self._last_sent_throttle = None
        self._last_sent_brake = None
        self._last_send_time_throttle = 0.0
        self._last_send_time_brake = 0.0

        self._estop = False
        self._gps_lat = None
        self._gps_lon = None
        self._imu_heading = None
        self._imu_roll = None
        self._imu_pitch = None

        self._interactive = sys.stdin.isatty()

        self._gnss_proc = None
        self._imu_proc = None
        self._lidar_proc = None
        self._foxglove_proc = None
        self._rsp_proc = None
        self._lidar_tf_proc = None
        self._camera_proc = None
        self._remote_last_time = 0.0

        self._cmd_vel_linear_x = 0.0
        self._cmd_vel_angular_z = 0.0

        self._hb_enabled = self.get_parameter('heartbeat_enabled').value
        self._hb_rate = self.get_parameter('heartbeat_rate').value
        self._hb_timeout = self.get_parameter('heartbeat_timeout').value
        self._hb_seq = 0
        self._hb_last_rx_time = None
        self._hb_link_lost = False
        self._hb_rx_count = 0
        self._hb_tx_count = 0
        self._hb_drop_count = 0
        self._hb_last_log_time = 0.0

        self._blue_enabled = self.get_parameter('blue').value and BLUE_AVAILABLE
        self._blue_loop = None
        self._blue_thread = None
        self._blue_state = None
        self._blue_config_store = None
        self._blue_service = None

        if self._blue_enabled:
            self._init_blue()

        self._joy_throttle = 0.0
        self._joy_brake = 0.0
        self._joy_steer = 0.0
        self._joy_active = False
        self._joy_last_time = 0.0

        # init all publishers, subscribers and drivers
        if self._remote_mode:
            self._teleop_pub = self.create_publisher(Twist, '/radio/teleop', 10)
            self._gear_pub = self.create_publisher(Int32, '/radio/gear', 10)
            if self._hb_enabled:
                self._hb_pub = self.create_publisher(UInt32, '/radio/heartbeat', 10)
                self.create_timer(1.0 / self._hb_rate, self._hb_send)
            if self._launch_sensors:
                self._init_foxglove_bridge()
                self._init_robot_state_publisher()
            self.create_subscription(NavSatFix, '/navsatfix', self._gps_cb, 10)
            self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        else:
            if self._launch_sensors:
                self._init_gnss_driver()
                self._init_imu_driver()
                self._init_lidar_driver()
                self._init_camera_driver()
                self._init_foxglove_bridge()
                self._init_robot_state_publisher()
            self._init_lss()
            self._init_phidgets()
            self._init_gear_servo()
            self.create_subscription(NavSatFix, '/navsatfix', self._gps_cb, 10)
            self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
            self.create_subscription(Twist, '/radio/teleop', self._remote_teleop_cb, 10)
            self.create_subscription(Int32, '/radio/gear', self._remote_gear_cb, 10)
            if self._hb_enabled:
                self.create_subscription(UInt32, '/radio/heartbeat', self._hb_recv, 10)
                self.create_timer(0.1, self._hb_watchdog)

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self._gui_mode = self.get_parameter('gui').value and DEARPYGUI_AVAILABLE

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self.create_timer(0.02, self._tick)
        if not self._remote_mode:
            self.create_timer(2.0, self._reconnect_check)

    """
    Subprocess Launchers: this is basically just for the sensors

    Essentially each driver spawns an external ROS2 process
    All processes will be ended in the destroy_node func after

    """

    def _init_gnss_driver(self):
        ws = os.path.expanduser('~/exia_ws')
        candidates = [
            os.path.join(ws, 'src', 'exia_bringup', 'config', 'septentrio_rover.yaml'),
            os.path.join(ws, 'install', 'exia_bringup', 'share',
                         'exia_bringup', 'config', 'septentrio_rover.yaml'),
        ]
        params_file = None
        for c in candidates:
            if os.path.isfile(c):
                params_file = c
                break
        if params_file is None:
            self.get_logger().error(f'GPS params file not found, tried: {candidates}')
            return
        try:
            self._gnss_proc = subprocess.Popen(
                ['ros2', 'run', 'septentrio_gnss_driver',
                 'septentrio_gnss_driver_node',
                 '--ros-args', '--params-file', params_file],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'GPS driver started (pid={self._gnss_proc.pid})')
        except Exception as e:
            self.get_logger().warn(f'Failed to start GPS driver: {e}')

    def _init_imu_driver(self):
        try:
            self._imu_proc = subprocess.Popen(
                ['ros2', 'run', 'exia_driver', 'imu_node'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'IMU driver started (pid={self._imu_proc.pid})')
        except Exception as e:
            self.get_logger().warn(f'Failed to start IMU driver: {e}')

    def _init_lidar_driver(self):
        try:
            self._lidar_proc = subprocess.Popen(
                ['ros2', 'launch', 'velodyne',
                 'velodyne-all-nodes-VLP32C-launch.py'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'Lidar driver started (pid={self._lidar_proc.pid})')
        except Exception as e:
            self.get_logger().warn(f'Failed to start lidar driver: {e}')
        try:
            self._lidar_tf_proc = subprocess.Popen(
                ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '--x', '0', '--y', '0', '--z', '0',
                 '--roll', '0', '--pitch', '0', '--yaw', '0',
                 '--frame-id', 'lidar_link', '--child-frame-id', 'velodyne'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info('Lidar TF (lidar_link->velodyne) published')
        except Exception as e:
            self.get_logger().warn(f'Failed to start lidar TF publisher: {e}')

    def _init_camera_driver(self):
        try:
            self._camera_proc = subprocess.Popen(
                ['ros2', 'run', 'orbbec_camera', 'orbbec_camera_node',
                 '--ros-args',
                 '-p', 'enable_point_cloud:=false',
                 '-p', 'enable_colored_point_cloud:=false'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'Camera driver started (pid={self._camera_proc.pid})')
        except Exception as e:
            self.get_logger().warn(f'Failed to start camera driver: {e}')

    def _init_foxglove_bridge(self):
        try:
            self._foxglove_proc = subprocess.Popen(
                ['ros2', 'launch', 'foxglove_bridge',
                 'foxglove_bridge_launch.xml', 'port:=8765'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'Foxglove bridge started on :8765 (pid={self._foxglove_proc.pid})')
        except Exception as e:
            self.get_logger().warn(f'Failed to start Foxglove bridge: {e}')

    def _init_robot_state_publisher(self):
        urdf_xacro = os.path.join(
            os.path.expanduser('~/exia_ws'),
            'src', 'exia_bringup', 'urdf', 'exia_ground.urdf.xacro')
        if not os.path.isfile(urdf_xacro):
            self.get_logger().warn(f'URDF not found: {urdf_xacro}')
            return
        try:
            result = subprocess.run(
                ['xacro', urdf_xacro],
                capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                self.get_logger().warn(f'xacro failed: {result.stderr.strip()}')
                return
            urdf_xml = result.stdout
            self._rsp_proc = subprocess.Popen(
                ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                 '--ros-args', '-p', f'robot_description:={urdf_xml}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'Robot state publisher started (pid={self._rsp_proc.pid})')
        except Exception as e:
            self.get_logger().warn(f'Failed to start robot_state_publisher: {e}')

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

    # for steering motor (position control via DCC1000 + HKT22 encoder)
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
            self.get_logger().info(
                f'MotorPositionController attached (max_degrees={self._motor_degrees_at_max_steer}, '
                f'current_limit={self.get_parameter("steering_current_limit").value}A)')
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

    def _set_steering_position(self, normalized):
        with self._motor_lock:
            if not self._motor_ok or self._phidgets_controller is None:
                return
            ctrl = self._phidgets_controller
            try:
                target = normalized * self._motor_degrees_at_max_steer
                ctrl.setTargetPosition(target)
                ctrl.resetFailsafe()
                duty = abs(ctrl.getDutyCycle())
                if duty > 0.95:
                    self._phidgets_runaway_count += 1
                    if self._phidgets_runaway_count > 25:
                        self.get_logger().error('Steering motor runaway detected — disengaging')
                        ctrl.setEngaged(False)
                        self._motor_ok = False
                else:
                    self._phidgets_runaway_count = 0
            except Exception:
                pass

    def _init_gear_servo(self):
        if not JETSON_GPIO_AVAILABLE:
            return
        if self._gear_ok:
            return
        try:
            pin = self.get_parameter('gear_pwm_pin').value
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(pin, GPIO.OUT)
            self._gear_pwm = GPIO.PWM(pin, GEAR_PWM_FREQ)
            self._gear_pwm.start(self._gear_pwm_map[1])
            self._gear_ok = True
            self.get_logger().info(
                f'Gear servo on pin {pin}: OK '
                f'(R={self._gear_pwm_map[0]}% N={self._gear_pwm_map[1]}% H={self._gear_pwm_map[2]}%)')
        except Exception as e:
            self.get_logger().warn(f'Gear servo init failed: {e}')

    def _reconnect_check(self):
        if not self._lss_ok and self._lss_pending is None:
            self._init_lss()
            if self._lss_ok:
                self.get_logger().info('LSS reconnected')

        if not self._gear_ok:
            self._init_gear_servo()
            if self._gear_ok:
                self.get_logger().info('Gear servo reconnected')

        with self._motor_lock:
            motor_ok = self._motor_ok
        if not motor_ok and self._phidgets_controller is None:
            self._init_phidgets()

    def _request_gear(self, target, force=False):
        with self._gear_lock:
            if target == self._gear:
                return
            now = time.monotonic()
            if not force:
                if now - self._last_gear_shift_time < 0.3:
                    return
                if target != 1 and (self._throttle_ramp > 0.05 or self._brake_ramp < 0.3):
                    return
            if self._gear == 0 and target == 2:
                return
            if self._gear == 2 and target == 0:
                return
            self._gear = target
            self._last_gear_shift_time = now
        self._set_gear(target)

    def _set_gear(self, gear_index):
        if self._remote_mode:
            gear_msg = Int32()
            gear_msg.data = gear_index
            self._gear_pub.publish(gear_msg)
            return
        if self._gear_pwm is not None and self._gear_ok:
            duty = self._gear_pwm_map.get(gear_index)
            if duty is not None:
                self._gear_pwm.ChangeDutyCycle(duty)

    def _gps_cb(self, msg):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self._gps_lat = msg.latitude
            self._gps_lon = msg.longitude

    # Return IMU heading
    def _imu_cb(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._imu_heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._imu_roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._imu_pitch = math.degrees(math.asin(sinp))

    # joystick callback
    def _joy_cb(self, msg):
        if len(msg.axes) < 4 or len(msg.buttons) < 4:
            return

        left_y = msg.axes[1]
        right_x = msg.axes[3]
        l2 = msg.axes[2] if len(msg.axes) > 2 else 1.0
        r2 = msg.axes[5] if len(msg.axes) > 5 else 1.0

        self._joy_throttle = _clamp((1.0 - r2) / 2.0, 0.0, 1.0)
        self._joy_brake = _clamp((1.0 - l2) / 2.0, 0.0, 1.0)
        self._joy_steer = _clamp(right_x, -1.0, 1.0)

        if left_y > 0.15:
            self._joy_throttle = _clamp(left_y, 0.0, 1.0)
        elif left_y < -0.15:
            self._joy_brake = _clamp(-left_y, 0.0, 1.0)

        has_input = (self._joy_throttle > 0.02
                     or self._joy_brake > 0.02
                     or abs(self._joy_steer) > 0.02)
        if has_input:
            self._joy_active = True
            self._joy_last_time = time.monotonic()
        else:
            self._joy_active = False

        if msg.buttons[0]:
            with self._gear_lock:
                target = max(self._gear - 1, 0)
            self._request_gear(target)
        elif msg.buttons[2]:
            with self._gear_lock:
                target = min(self._gear + 1, 2)
            self._request_gear(target)
        elif msg.buttons[3]:
            self._request_gear(1)
            if self._estop:
                self._estop = False

        if msg.buttons[1]:
            self._safe_stop()

    def _remote_teleop_cb(self, msg):
        if self._estop:
            return
        self._throttle_ramp = _clamp(msg.linear.x, 0.0, 1.0)
        self._brake_ramp = _clamp(msg.linear.y, 0.0, 1.0)
        self._steer_ramp = _clamp(msg.angular.z, -1.0, 1.0)
        self._remote_last_time = time.monotonic()

    def _remote_gear_cb(self, msg):
        gear = _clamp(msg.data, 0, 2)
        self._request_gear(gear, force=(gear == 1))

    def _publish_cmd_vel(self):
        with self._gear_lock:
            gear = self._gear
        if gear == 2:
            speed = self._throttle_ramp * CMD_VEL_MAX_SPEED
        elif gear == 0:
            speed = -self._throttle_ramp * CMD_VEL_MAX_REVERSE
        else:
            speed = 0.0

        if self._brake_ramp > 0.05:
            speed *= max(0.0, 1.0 - self._brake_ramp)

        steer_angle = self._steer_ramp * CMD_VEL_MAX_STEER

        msg = Twist()
        msg.linear.x = speed
        if abs(speed) > 0.05 and abs(steer_angle) > 0.001:
            msg.angular.z = speed / (CMD_VEL_WHEELBASE / math.tan(steer_angle))
        else:
            msg.angular.z = 0.0
        self._cmd_vel_linear_x = msg.linear.x
        self._cmd_vel_angular_z = msg.angular.z
        self._cmd_vel_pub.publish(msg)

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick_t
        self._last_tick_t = now

        if self._lss_pending is not None and now - self._lss_open_time >= 0.5:
            self._init_lss_finish()

        if self._estop:
            if not self._remote_mode:
                self._send_lss(f'#{LSS_THROTTLE_ID}D{LSS_THROTTLE_NEUTRAL}\r')
                self._send_lss(f'#{LSS_BRAKE_ID}D{LSS_BRAKE_ENGAGED}\r')
                self._set_steering_position(0.0)
                stop_msg = Twist()
                self._cmd_vel_pub.publish(stop_msg)
            self._print_hud()
            return

        remote_active = (not self._remote_mode) and (now - self._remote_last_time) < 0.3
        joy_active = self._joy_active or (now - self._joy_last_time) < 0.3

        if joy_active and not remote_active:
            self._throttle_ramp = self._joy_throttle
            self._brake_ramp = self._joy_brake
            self._steer_ramp = self._joy_steer
        elif not remote_active:
            self._throttle_ramp = _clamp(self._throttle_ramp - SPRING_RATE * dt, 0.0, 1.0)
            self._brake_ramp = _clamp(self._brake_ramp - SPRING_RATE * dt, 0.0, 1.0)
            if self._steer_ramp > 0.0:
                self._steer_ramp = _clamp(
                    self._steer_ramp - STEER_SPRING_RATE * dt, 0.0, 1.0)
            elif self._steer_ramp < 0.0:
                self._steer_ramp = _clamp(
                    self._steer_ramp + STEER_SPRING_RATE * dt, -1.0, 0.0)

        self._publish_cmd_vel()

        if self._remote_mode:
            msg = Twist()
            msg.linear.x = self._throttle_ramp
            msg.linear.y = self._brake_ramp
            msg.angular.z = self._steer_ramp
            self._teleop_pub.publish(msg)
            self._print_hud()
            return

        thr_target = _lerp(LSS_THROTTLE_NEUTRAL, LSS_THROTTLE_MAX, self._throttle_ramp)
        brk_target = _lerp(LSS_BRAKE_RELEASED, LSS_BRAKE_ENGAGED, self._brake_ramp)

        thr_val = self._throttle_f.update(thr_target, dt)
        brk_val = self._brake_f.update(brk_target, dt)

        self._set_steering_position(self._steer_ramp)

        thr_int = int(round(thr_val))
        thr_at_neutral = (thr_int == LSS_THROTTLE_NEUTRAL)
        thr_needs_send = (
            self._last_sent_throttle is None
            or abs(thr_int - self._last_sent_throttle) >= LSS_SEND_DEADBAND
            or (thr_at_neutral and self._last_sent_throttle != LSS_THROTTLE_NEUTRAL))
        if thr_needs_send and (now - self._last_send_time_throttle) >= LSS_SEND_MIN_INTERVAL:
            self._send_lss(f'#{LSS_THROTTLE_ID}D{thr_int}T50\r')
            self._last_sent_throttle = thr_int
            self._last_send_time_throttle = now

        brk_int = int(round(brk_val))
        brk_at_released = (brk_int == LSS_BRAKE_RELEASED)
        brk_at_engaged = (brk_int == LSS_BRAKE_ENGAGED)
        brk_needs_send = (
            self._last_sent_brake is None
            or abs(brk_int - self._last_sent_brake) >= LSS_SEND_DEADBAND
            or (brk_at_released and self._last_sent_brake != LSS_BRAKE_RELEASED)
            or (brk_at_engaged and self._last_sent_brake != LSS_BRAKE_ENGAGED))
        if brk_needs_send and (now - self._last_send_time_brake) >= LSS_SEND_MIN_INTERVAL:
            self._send_lss(f'#{LSS_BRAKE_ID}D{brk_int}T50\r')
            self._last_sent_brake = brk_int
            self._last_send_time_brake = now

        self._print_hud()

    def _safe_stop(self):
        self._estop = True
        self._throttle_ramp = 0.0
        self._brake_ramp = 1.0
        self._steer_ramp = 0.0
        self._request_gear(1, force=True)
        stop_msg = Twist()
        self._cmd_vel_pub.publish(stop_msg)
        if self._remote_mode:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 1.0
            msg.angular.z = 0.0
            self._teleop_pub.publish(msg)
        else:
            self._throttle_f.snap(LSS_THROTTLE_NEUTRAL)
            self._brake_f.snap(LSS_BRAKE_ENGAGED)
            self._last_sent_throttle = None
            self._last_sent_brake = None
            self._send_lss(f'#{LSS_THROTTLE_ID}D{LSS_THROTTLE_NEUTRAL}\r')
            self._send_lss(f'#{LSS_BRAKE_ID}D{LSS_BRAKE_ENGAGED}\r')
            self._set_steering_position(0.0)
        self._print_hud()

    def _hb_send(self):
        self._hb_seq += 1
        self._hb_tx_count += 1
        msg = UInt32()
        msg.data = self._hb_seq
        self._hb_pub.publish(msg)

    def _hb_recv(self, msg):
        self._hb_last_rx_time = time.monotonic()
        self._hb_rx_count += 1
        if self._hb_link_lost:
            self._hb_link_lost = False
            self._estop = False
            self.get_logger().info('Link restored, heartbeat-loss e-stop auto-cleared')

    def _hb_watchdog(self):
        if self._hb_last_rx_time is None:
            return
        elapsed = time.monotonic() - self._hb_last_rx_time
        if elapsed > self._hb_timeout and not self._hb_link_lost:
            self._hb_link_lost = True
            self._hb_drop_count += 1
            self.get_logger().warn(f'Heartbeat lost ({elapsed:.2f}s), triggering e-stop')
            self._safe_stop()
        now = time.monotonic()
        if now - self._hb_last_log_time >= 10.0:
            self._hb_last_log_time = now
            self.get_logger().info(
                f'[Heartbeat] rx={self._hb_rx_count} drops={self._hb_drop_count} '
                f'link={"OK" if not self._hb_link_lost else "LOST"}')

    def _signal_handler(self, _sig, _frame):
        self._safe_stop()
        raise SystemExit(0)

    def _throttle_pct(self):
        return int(self._throttle_ramp * 100)

    def _brake_pct(self):
        return int(self._brake_ramp * 100)

    def _print_hud(self):
        if not self._interactive or self._gui_mode:
            return

        gear_str = GEAR_NAMES.get(self._gear, '?')
        duty_pct = int(self._steer_ramp * 100)
        estop_str = ' ** E-STOP **' if self._estop else ''

        sys.stdout.write('\033[2A\r\033[K')

        if self._remote_mode:
            sys.stdout.write(
                f'[REMOTE] '
                f'T:{self._throttle_pct():3d}% '
                f'B:{self._brake_pct():3d}% '
                f'G:{gear_str} '
                f'S:{duty_pct:+4d}%'
                f'{estop_str}'
            )
            if self._gps_lat is not None:
                gps_str = f'{self._gps_lat:.6f},{self._gps_lon:.6f}'
            else:
                gps_str = '--'
            if self._imu_heading is not None:
                imu_str = f'H:{self._imu_heading:+.0f} R:{self._imu_roll:+.0f} P:{self._imu_pitch:+.0f}'
            else:
                imu_str = '--'
            sys.stdout.write(f'\n\033[K GPS:{gps_str} IMU:{imu_str}\n')
        else:
            with self._motor_lock:
                motor_ok = self._motor_ok
            lss_tag = 'LSS:OK' if self._lss_ok else 'LSS:--'
            steer_tag = 'MTR:OK' if motor_ok else 'MTR:--'
            remote_active = (time.monotonic() - self._remote_last_time) < 0.3
            if self._hb_enabled and self._hb_link_lost:
                src_tag = ' [LINK LOST]'
            elif remote_active:
                src_tag = ' [WIFI]'
            else:
                src_tag = ''
            sys.stdout.write(
                f'[{lss_tag} {steer_tag}] '
                f'T:{self._throttle_pct():3d}% '
                f'B:{self._brake_pct():3d}% '
                f'G:{gear_str} '
                f'S:{duty_pct:+4d}%'
                f'{estop_str}{src_tag}'
            )
            if self._gps_lat is not None:
                gps_str = f'{self._gps_lat:.6f},{self._gps_lon:.6f}'
            else:
                gps_str = '--'
            if self._imu_heading is not None:
                imu_str = f'H:{self._imu_heading:+7.1f} R:{self._imu_roll:+6.1f} P:{self._imu_pitch:+6.1f}'
            else:
                imu_str = '--'
            sys.stdout.write(f'\n\033[K GPS:{gps_str} IMU:{imu_str}\n')

        sys.stdout.flush()

    def destroy_node(self):
        self._safe_stop()

        for proc in [self._gnss_proc, self._imu_proc, self._lidar_proc,
                     self._camera_proc, self._lidar_tf_proc,
                     self._foxglove_proc, self._rsp_proc]:
            if proc is not None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()

        if not self._remote_mode:
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
                    self._gear_pwm.stop()
                    GPIO.cleanup()
                except Exception:
                    pass

        sys.stdout.write('\n')

        if self._blue_loop is not None:
            self._blue_loop.call_soon_threadsafe(self._blue_loop.stop)
        if self._blue_thread is not None:
            self._blue_thread.join(timeout=5)

        super().destroy_node()

    def _init_blue(self):
        import asyncio
        from pathlib import Path
        from exia_control.blue_core.config import ConfigStore
        from exia_control.blue_core.state import RuntimeState
        from exia_control.argus.exia_ros_vehicle import ExiaRosVehicle

        config_path = Path.home() / '.exia' / 'blue_config.toml'
        config_path.parent.mkdir(parents=True, exist_ok=True)
        self._blue_config_store = ConfigStore.load_or_create(config_path)
        self._blue_state = RuntimeState()
        self._blue_vehicle = ExiaRosVehicle(self)

        self._blue_loop = asyncio.new_event_loop()
        self._blue_thread = threading.Thread(target=self._run_blue_loop, daemon=True)
        self._blue_thread.start()
        self.get_logger().info('Blue sync started (config: %s)' % str(config_path))

    def _run_blue_loop(self):
        import asyncio
        asyncio.set_event_loop(self._blue_loop)
        self._blue_loop.run_until_complete(self._blue_main())

    async def _blue_main(self):
        import asyncio
        import logging
        from exia_control.blue.client import BlueClient
        from exia_control.blue_core.service import AppService

        cfg = self._blue_config_store.get()
        logger = logging.getLogger('exia.blue')
        async with BlueClient(cfg.blue_base_url, cfg.blue_api_key, cfg.request_timeout_seconds) as client:
            self._blue_service = AppService(
                config_store=self._blue_config_store,
                state=self._blue_state,
                client=client,
                vehicle=self._blue_vehicle,
                logger=logger,
            )
            await self._blue_service.start()
            try:
                while rclpy.ok():
                    await asyncio.sleep(0.1)
            except asyncio.CancelledError:
                pass
            finally:
                await self._blue_service.stop()


def main(args=None):
    rclpy.init(args=args)

    node = HwTeleopNode()
    gui_requested = node.get_parameter('gui').value

    if gui_requested and DEARPYGUI_AVAILABLE:
        from exia_control.gui.dashboard import create_dashboard, run_frame, stop_dashboard
        create_dashboard(
            node,
            blue_state=node._blue_state,
            blue_config_store=node._blue_config_store,
            blue_service=node._blue_service,
        )
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        try:
            while run_frame():
                pass
        except (KeyboardInterrupt, SystemExit):
            pass
        finally:
            stop_dashboard()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
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
