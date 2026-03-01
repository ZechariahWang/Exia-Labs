import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

HEADER = b'\xAA\x55'
VALID_LENGTHS = {0x10, 0x14, 0x2C}
G = 9.80665


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self._declare_parameters()

        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.output_rate = self.get_parameter('output_rate').value

        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.buf = bytearray()
        self.max_buf_size = 4096

        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.angle = [0.0, 0.0, 0.0]
        self.last_print_time = 0.0

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        self._init_serial()

        period = 1.0 / self.output_rate
        self.read_timer = self.create_timer(period, self._read_loop)
        self.reconnect_timer = self.create_timer(2.0, self._try_reconnect)

        self.get_logger().info(
            f'IMU node started on {self.port} at {self.baud_rate} baud'
        )

    def _declare_parameters(self):
        self.declare_parameter('port', '/dev/exia_imu')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('output_rate', 200)

    def _init_serial(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            time.sleep(0.5)
            self.serial_conn.reset_input_buffer()
            self.buf.clear()
            self.get_logger().info(f'Serial connected: {self.port}')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.serial_conn = None

    def _try_reconnect(self):
        if self.serial_conn is not None:
            return
        self.get_logger().info(f'Attempting reconnect to {self.port}...')
        self._init_serial()

    def _read_loop(self):
        if self.serial_conn is None:
            return

        try:
            with self.serial_lock:
                waiting = self.serial_conn.in_waiting
                if waiting > 0:
                    self.buf.extend(self.serial_conn.read(waiting))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial disconnected: {e}')
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
            return

        if len(self.buf) > self.max_buf_size:
            self.buf = self.buf[-self.max_buf_size:]

        publish = False

        while len(self.buf) >= 5:
            idx = self.buf.find(HEADER)
            if idx == -1:
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 3:
                break

            payload_len = self.buf[2]
            if payload_len not in VALID_LENGTHS:
                del self.buf[:2]
                continue

            packet_total = 2 + 1 + payload_len + 2
            if len(self.buf) < packet_total:
                break

            payload = self.buf[3:3 + payload_len]
            del self.buf[:packet_total]

            if payload_len == 0x2C:
                floats = struct.unpack_from('<11f', payload, 0)
                self.gyro[0] = floats[2]
                self.gyro[1] = floats[3]
                self.gyro[2] = floats[4]
                self.accel[0] = floats[5] * G
                self.accel[1] = floats[6] * G
                self.accel[2] = floats[7] * G

            elif payload_len == 0x14:
                floats = struct.unpack_from('<5f', payload, 0)
                self.angle[0] = floats[3]
                self.angle[1] = floats[2]
                self.angle[2] = floats[4]
                publish = True

        if publish:
            self._publish_imu()

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        q = self._euler_to_quaternion(
            math.radians(self.angle[0]),
            math.radians(self.angle[1]),
            math.radians(self.angle[2])
        )
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]

        msg.linear_acceleration.x = self.accel[0]
        msg.linear_acceleration.y = self.accel[1]
        msg.linear_acceleration.z = self.accel[2]

        msg.orientation_covariance[0] = 0.0025
        msg.orientation_covariance[4] = 0.0025
        msg.orientation_covariance[8] = 0.0025

        msg.angular_velocity_covariance[0] = 0.0003
        msg.angular_velocity_covariance[4] = 0.0003
        msg.angular_velocity_covariance[8] = 0.0003

        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(msg)

        now = time.monotonic()
        if now - self.last_print_time >= 0.2:
            self.last_print_time = now
            heading = self.angle[2]
            self.get_logger().info(
                f'Heading: {heading:7.2f}°  Roll: {self.angle[0]:6.2f}°  Pitch: {self.angle[1]:6.2f}°'
            )

    def _euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return (x, y, z, w)

    def destroy_node(self):
        self.get_logger().info('IMU node shutting down')
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()

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
