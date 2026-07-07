import rclpy
from rclpy.node import Node
import math
from time import time
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from std_msgs.msg import Int32MultiArray
import math
import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('servo_controller')
        self.publisher_1 = self.create_publisher(SetPosition, 'servo1/set_position', 10)
        self.publisher_2 = self.create_publisher(SetPosition, 'servo2/set_position', 10)
        self.publisher_3 = self.create_publisher(SetPosition, 'servo3/set_position', 10)
        self.publisher_4 = self.create_publisher(SetPosition, 'servo4/set_position', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tick_count = 0
        self.start_time = time()

        self.servo_1_position = 2047
        self.servo_2_position = 2047
        self.servo_3_position = 2047
        self.servo_4_position = 2047
        self.decreasing = True
        self.increasing = True


    def timer_callback(self):
        self.tick_count += 1

        msg_1 = SetPosition()
        msg_2 = SetPosition()
        msg_3 = SetPosition()
        msg_4 = SetPosition()

        msg_1.id = 1
        msg_2.id = 2
        msg_3.id = 3
        msg_4.id = 4

        msg_1.position = self.servo_1_position
        msg_2.position = self.servo_2_position
        msg_3.position = self.servo_3_position
        msg_4.position = self.servo_4_position
        
        self.publisher_1.publish(msg_1)
        self.publisher_2.publish(msg_2)
        self.publisher_3.publish(msg_3)
        self.publisher_4.publish(msg_4)

        # self.servo_1_position = self.servo_back(self.servo_1_position, 1500, 2048)
        #self.servo_2_position = self.servo_movement(self.servo_2_position, 1000, 0)
        
        # self.servo_3_position = self.servo_movement(self.servo_3_position, 1500, 0)
        # self.servo_4_position = self.servo_movement(self.servo_4_position, 1000, 900)
        
        self.servo_2_position = self.sin_servo_move(self.servo_2_position, 700, math.pi, 2048)
        self.servo_1_position = self.sin_servo_move(self.servo_1_position, 500, math.pi, 1000)
        # self.get_logger().info('Publishing: "%d"' % self.servo_1_position)



    def sin_servo_move(self, servo_position=0, amp=1000, omega=math.pi, offset=2500):
        
        elapsed_time = time() - self.start_time

        servo_position = round(amp * math.sin(omega * elapsed_time) + offset)

        return servo_position
    
    def servo_movement(self, servo_position=0, steps=1000, start=0):

        if servo_position >= steps + start:
            self.increasing = False
        elif servo_position <= start:
            self.increasing = True
        
        if self.increasing:
            servo_position += 3
        else:
            servo_position -= 5
        
        return servo_position
    
    def servo_back(self, servo_position=0, steps=1000, start=0):

        if servo_position <= start - steps:
            self.decreasing = False
        elif servo_position >= start:
            self.decreasing = True
        
        if self.decreasing:
            servo_position -= 5
        else:
            servo_position += 3
        
        return servo_position
        

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