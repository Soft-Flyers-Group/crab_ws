import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
import math
import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('servo_controller')
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        frequency = 0.1
        amplitude = 180.0

        counts_per_degree = 4096 / 360
        center = 2048

        t = time.time() - self.start_time

        angle = amplitude * math.sin(2 * math.pi * frequency * t)

        position = center + angle * counts_per_degree

        position = int(max(0, min(4095, position)))

        msg = SetPosition()
        msg.id = 1
        msg.position = self.i
        self.publisher_.publish(msg)
        self.i += 1
        if self.i >= 4096:
            self.i = 0


def main(args=None):
    rclpy.init(args=args)

    node = MinimalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()