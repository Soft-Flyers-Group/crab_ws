import time

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from apriltag_msgs.msg import AprilTagDetectionArray


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Publish to Servo Node to move the servo
        self.publisher_ = self.create_publisher(
            SetPosition, 'servo/set_position', 10
        )
        # Servo 1 is YAW, Servo 2 is ROLL
        self.servo_commands = [0,0]

        # Timer Call Back at 200 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)


    def timer_callback(self):

        # 0-4096 Range from 0 to 360 degrees
        # Travel full range for both servos
        if self.servo_commands[0] >= 4069:
            self.servo_commands[0] = 0
        else:
            self.servo_commands[0] += 10

        if self.servo_commands[1] >= 4069:
            self.servo_commands[1] = 0
        else:
            self.servo_commands[1] += 10

        for servo_id, command in enumerate(self.servo_commands, start=1):
            msg = SetPosition()
            msg.id = servo_id
            msg.position = int(command)
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()

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