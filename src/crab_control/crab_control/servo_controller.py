import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('servo_controller')
        self.publisher_ = self.create_publisher(SetPosition, 'servo/set_position', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = SetPosition()
        msg.id = 1
        msg.position = self.i
        self.publisher_.publish(msg)
        msg.id = 2
        self.publisher_.publish(msg)
        self.i += 50
        if self.i >= 4095:
            self.i = 0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()