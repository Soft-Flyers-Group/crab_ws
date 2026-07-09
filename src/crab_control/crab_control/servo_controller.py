import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from std_msgs.msg import Int32MultiArray
import math
import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('servo_controller')

        self.publisher_ = self.create_publisher(
            SetPosition,
            'servo/set_position',
            10
        )

        self.position_sub = self.create_subscription(
        Int32MultiArray,
        'servo/get_position',
        self.position_callback,
        10
        )

        self.current_positions = [0, 0]

        self.timer = self.create_timer(0.001, self.timer_callback)

        self.start_time = time.time()

        self.position = 2048
        self.position2 = 2048
        self.counter = 0

    def position_callback(self, msg):
        self.current_positions = list(msg.data)

        # Encoder value of servo 1
        encoder1 = self.current_positions[0]

        # Encoder value of servo 2
        encoder2 = self.current_positions[1]

        self.get_logger().info(
            f"Servo1: {encoder1}, Servo2: {encoder2}"
    )
        
    


    def timer_callback(self):

        frequency = 0.4
        amplitude = 60.0

        counts_per_degree = 4096 / 360
        center = 2048

        t = time.time() - self.start_time

        angle = amplitude * math.sin(2 * math.pi * frequency * t)

        position = int(center + angle * counts_per_degree)
        position = max(0, min(4095, position))

        msg = SetPosition()
        msg.id = 1
        msg.position = position

        enc = self.current_positions[0]

        

        if enc > 2648:
            self.counter = 1

        if self.counter == 1:
            self.position2 = 3048

        if enc < 1448:
            self.counter = 0
            self.position2 = 2048

        msg2 = SetPosition()
        msg2.id = 2
        msg2.position = self.position2

        self.publisher_.publish(msg)
        self.publisher_.publish(msg2)



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