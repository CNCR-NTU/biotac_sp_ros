import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from smbus2 import SMBus
import math
import time

class OrientationPublisher(Node):

    def __init__(self):
        super().__init__('orientation_publisher')

        self.publisher_ = self.create_publisher(Vector3, 'mpu9250/orientation', 10)
        self.timer = self.create_timer(0.1, self.publish_orientation)

        self.bus = SMBus(0)
        self.ADDR = 0x68
        self.GYRO_SENS = 131.0
        self.ALPHA = 0.2

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.last_time = time.time()

        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.PWR_MGMT_1 = 0x6B

        self.bus.write_byte_data(self.ADDR, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        self.get_logger().info("Calibrating gyroscope...")
        self.gyro_bias = self.calibrate_gyro()
        self.get_logger().info("Publishing mesages in topic mpu9250/orientation")

    def read_axis(self, reg_high):
        high = self.bus.read_byte_data(self.ADDR, reg_high)
        low = self.bus.read_byte_data(self.ADDR, reg_high + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value

    def calibrate_gyro(self):
        bias = [0.0, 0.0, 0.0]
        for _ in range(100):
            bias[0] += self.read_axis(self.GYRO_XOUT_H)
            bias[1] += self.read_axis(self.GYRO_XOUT_H + 2)
            bias[2] += self.read_axis(self.GYRO_XOUT_H + 4)
            time.sleep(0.01)
        return [x / 100 for x in bias]

    def publish_orientation(self):
        acc_x = self.read_axis(self.ACCEL_XOUT_H)
        acc_y = self.read_axis(self.ACCEL_XOUT_H + 2)
        acc_z = self.read_axis(self.ACCEL_XOUT_H + 4)

        gyro_x = (self.read_axis(self.GYRO_XOUT_H) - self.gyro_bias[0]) / self.GYRO_SENS
        gyro_y = (self.read_axis(self.GYRO_XOUT_H + 2) - self.gyro_bias[1]) / self.GYRO_SENS
        gyro_z = (self.read_axis(self.GYRO_XOUT_H + 4) - self.gyro_bias[2]) / self.GYRO_SENS

        ax = acc_x / 16384.0
        ay = acc_y / 16384.0
        az = acc_z / 16384.0

        pitch_new = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
        roll_new = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        yaw_new = self.yaw + gyro_z * dt

        self.pitch = self.ALPHA * pitch_new + (1 - self.ALPHA) * self.pitch
        self.roll = self.ALPHA * roll_new + (1 - self.ALPHA) * self.roll
        self.yaw = self.ALPHA * yaw_new + (1 - self.ALPHA) * self.yaw

        msg = Vector3()
        msg.x = self.pitch
        msg.y = self.roll
        msg.z = self.yaw

        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published [Pitch: {msg.x:.2f}, Roll: {msg.y:.2f}, Yaw: {msg.z:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    node = OrientationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
