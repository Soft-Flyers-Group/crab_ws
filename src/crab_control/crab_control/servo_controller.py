# imports
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32MultiArray
import numpy as np
import time
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from std_msgs.msg import Int32MultiArray
import math
import time


class MinimalPublisher(Node):

    def __init__(self):

        # Initialization of publisher
        super().__init__('servo_controller')
        self.publisher_ = self.create_publisher(SetPosition, 'servo/set_position', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        
        # Initialization of subscriber to encoder values
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/servo/encoder_data',
            self.listener_callback,
            10)
        self.subscription

        # Time tracker
        self.start_time = time.time()

        # Initial Servo positions
        self.servo_1_init = 2100
        self.servo_2_init = 1638
        self.servo_3_init = 2048
        self.servo_4_init = 2048

        self.servo_1_position = self.servo_1_init
        self.servo_2_position = self.servo_2_init
        self.servo_3_position = self.servo_3_init
        self.servo_4_position = self.servo_4_init

        self.latest_positions = [0, 0]

        # Variables for linear servo movement
        self.decreasing = True
        self.increasing = True


    # recieving encoder values and storing in class variable
    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % str(msg.data))
        self.latest_positions = list(msg.data)

    def timer_callback(self):

        # Defining servo messages and IDs
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
        
        self.publisher_.publish(msg_1)
        self.publisher_.publish(msg_2)
        # self.publisher_.publish(msg_3)
        # self.publisher_.publish(msg_4)
        
        # Old servo movement commands
        # self.servo_3_position = self.linear_servo_move(self.servo_3_position, 1500, 0)
        # self.servo_4_position = self.linear_servo_move(self.servo_4_position, 1000, 900)
        
        # self.servo_2_position = self.sin_servo_move(self.servo_2_position, 700, math.pi, 2048)
        # self.servo_1_position = self.fourier_servo_move(self.servo_1_position, 500, math.pi, 1200)
        
        # self.servo_1_position = self.fourier_servo_move(self.servo_1_position, 500, math.pi, 2048)


        # CURRENT SERVO MOVEMENT CODE
        FLAP_RANGE = 600

        self.servo_1_position = self.fourier_servo_move(self.servo_1_position, FLAP_RANGE, math.pi, self.servo_1_init)

        if self.latest_positions[0] > self.servo_1_init + FLAP_RANGE - 150:
            self.servo_2_position = 2000
        if self.latest_positions[0] < self.servo_1_init - FLAP_RANGE + 150:
            self.servo_2_position = 1000
        
        self.get_logger().info('Publishing: "%s"' % str(self.latest_positions))

    # using Fourier series to define servo movement emulating piecewise on-off
    def fourier_servo_move(self, servo_position=0,amp=1000, omega=math.pi, offset=2048):

        elapsed_time = time.time() - self.start_time

        n = np.arange(1, 100) # creates array [1, 2, ... a]
        signs = (-1) ** (n + 1)

        # calculates terms of fourier series
        fourier_series = signs * 4 / math.pi * amp * np.cos((2*n - 1) * omega * elapsed_time) / n
        servo_position = round(np.sum(fourier_series)/5) * 5 + offset

        return servo_position

    # using a sinusoidal function to define back and forth servo movement
    def sin_servo_move(self, servo_position=0, amp=1000, omega=math.pi, offset=2048):
        
        elapsed_time = time.time() - self.start_time

        servo_position = round(amp * math.cos(omega * elapsed_time) + offset)

        return servo_position
    
    # using a linear addition to define back and forth servo momement
    def linear_servo_movem(self, servo_position=0, steps=1000, start=0):

        if servo_position >= steps + start:
            self.increasing = False
        elif servo_position <= start:
            self.increasing = True
        
        if self.increasing:
            servo_position += 3
        else:
            servo_position -= 5
        
        return servo_position

# publishing messages to the servos
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