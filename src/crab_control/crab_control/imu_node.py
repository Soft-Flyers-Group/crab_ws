#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import board
import busio
import adafruit_icm20x


class ICM20948Node(Node):

    def __init__(self):
        super().__init__('icm20948_node')

        self.publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        # Initialize I2C
        self.i2c = busio.I2C(
            board.SCL,
            board.SDA
        )

        # Initialize ICM-20948
        self.icm = adafruit_icm20x.ICM20948(self.i2c)

        # Publish at 100Hz
        self.timer = self.create_timer(
            0.01,
            self.publish_imu
        )

        self.get_logger().info(
            "ICM-20948 ROS2 node started"
        )


    def publish_imu(self):

        msg = Imu()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"


        # Read raw sensor data
        ax, ay, az = self.icm.acceleration
        gx, gy, gz = self.icm.gyro

        # Apply 180 degree rotation about Y axis

        # Acceleration
        msg.linear_acceleration.x = -ax
        msg.linear_acceleration.y =  ay
        msg.linear_acceleration.z = -az

        # Gyroscope
        msg.angular_velocity.x = -gx
        msg.angular_velocity.y =  gy
        msg.angular_velocity.z = -gz

        # ==========================
        # Orientation unavailable
        # ==========================

        msg.orientation_covariance[0] = -1

        # ==========================
        # Covariance estimates
        # (adjust later if calibrated)
        # ==========================

        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]


        self.publisher.publish(msg)



def main(args=None):

    rclpy.init(args=args)

    node = ICM20948Node()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
