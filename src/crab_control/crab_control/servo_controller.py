# imports
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32MultiArray
import numpy as np
import time
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from crab_interfaces.msg import ServoData
from apriltag_msgs.msg import AprilTagDetectionArray
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
        self.encoder_sub = self.create_subscription(
            ServoData,
            '/servo/encoder_data',
            self.encoder_callback,
            10)
        
        # Subscription to april tag detection data
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10)

        # Time tracker
        self.start_time = time.time()

        # Initial Servo positions
        self.servo_1_init = 2500
        self.servo_2_init = 2000
        self.servo_3_init = 1900
        self.servo_4_init = 1900

        self.servo_1_position = self.servo_1_init
        self.servo_2_position = self.servo_2_init
        self.servo_3_position = self.servo_3_init
        self.servo_4_position = self.servo_4_init

        self.latest_positions = [0, 0, 0, 0] # change for 4 servos

        # initialization for april tag detection values
        self.tag_id = 0
        self.cx = 0
        self.cy = 0
        self.corners = [[0, 0, 0, 0], [0, 0, 0, 0]]
        self.size_x = 0
        self.size_y = 0
        self.size = 0

        # Variables for linear servo movement
        self.decreasing = True
        self.increasing = True


        # Error margin for april tag localization
        self.error = 80


    # recieving encoder values and storing in class variable
    def encoder_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % str(msg.data))
        self.latest_positions = list(msg.data)

    # receiving april tag detection data
    def tag_callback(self, msg):
        if not msg.detections:
            return
        
        detection = msg.detections[0]

        self.tag_id = detection.id
        self.cx = detection.centre.x
        self.cy = detection.centre.y
        self.corners = np.array([
            [c.x for c in detection.corners],
            [c.y for c in detection.corners]
        ])

        self.size_x = self.corners[0][1] - self.corners[0][0]
        self.size_y = self.corners[1][0] - self.corners[1][1]
        self.size = np.sqrt(self.size_x ** 2 + self.size_y ** 2)

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
        self.publisher_.publish(msg_3) # change for 4 servos
        self.publisher_.publish(msg_4) # change for 4 servos
        

        # CURRENT SERVO MOVEMENT CODE
        # FLAP_RANGE = 400
        # self.servo_1_position = self.nestedsin_servo_move(self.servo_1_position, FLAP_RANGE, 0.7, self.servo_1_init, 1, 3)
        # self.servo_3_position = self.nestedsin_servo_move(self.servo_3_position, FLAP_RANGE, 0.7, self.servo_3_init, -1, 3)

        # self.servo_2_position = self.sin_servo_move(self.servo_2_position, 500, math.pi/2, self.servo_2_init - 500)
        # self.servo_4_position = self.sin_servo_move(self.servo_4_position, 500, math.pi/2, self.servo_4_init + 500, -1)


        # if self.latest_positions[0] > self.servo_1_init + FLAP_RANGE - 100:
        #     self.servo_2_position = self.servo_2_init - 1000
        # if self.latest_positions[0] < self.servo_1_init - FLAP_RANGE + 100:
        #     self.servo_2_position = self.servo_2_init
       # 
        # if self.latest_positions[2] > self.servo_3_init + FLAP_RANGE - 150:
        #     self.servo_4_position = self.servo_4_init
        # if self.latest_positions[2] < self.servo_3_init - FLAP_RANGE + 150:
        #     self.servo_4_position = self.servo_4_init + 1000
        
        FLAP_RANGE = 300

        if not (640 - self.error < self.cx < 640 + self.error):

            if self.cx < 640:
                direction = 1000
            else:
                direction = 0
            
            self.servo_1_position = self.sin_servo_move(self.servo_1_position, FLAP_RANGE, math.pi, self.servo_1_init, 1)
            self.servo_3_position = self.sin_servo_move(self.servo_3_position, FLAP_RANGE, math.pi, self.servo_3_init, 1)

            if self.latest_positions[0] > self.servo_1_init + FLAP_RANGE - 100:
                self.servo_2_position = self.servo_2_init - 1000 + direction
            if self.latest_positions[0] < self.servo_1_init - FLAP_RANGE + 100:
                self.servo_2_position = self.servo_2_init - direction
        
            if self.latest_positions[2] > self.servo_3_init + FLAP_RANGE - 150:
                self.servo_4_position = self.servo_4_init - 1000 + direction
            if self.latest_positions[2] < self.servo_3_init - FLAP_RANGE + 150:
                self.servo_4_position = self.servo_4_init - direction

        

        self.get_logger().info('Publishing: "%s"' % str(self.cx))


    # using the sum of sin functions to move back fast and forward slowly
    def nestedsin_servo_move(self, servo_position=0, amp=1000, omega=0.6, offset=2048, direction=1, speed=1):
        elapsed_time = (time.time() - self.start_time) * speed
        servo_position = round(direction * amp * math.sin(elapsed_time + omega * math.sin(elapsed_time))) + offset
        return servo_position
    
    def slowfast_servo_move(self, servo_position=0, amp=1000, omega=math.pi, offset=2048, direction=1):
        elapsed_time = time.time() - self.start_time

        n = np.arange(1, 100)
        sin_series = amp * np.sin(n * elapsed_time * omega) / n
        servo_position = round(np.sum(sin_series)/5) * 5 * direction + offset

        return servo_position

    # using Fourier series to define servo movement emulating piecewise on-off
    def fourier_servo_move(self, servo_position=0,amp=1000, omega=math.pi, offset=2048, direction=1):

        elapsed_time = time.time() - self.start_time

        n = np.arange(1, 300) # creates array [1, 2, ... a]
        signs = (-1) ** (n + 1)

        # calculates terms of fourier series
        fourier_series = signs * 4 / math.pi * amp * np.cos((2*n - 1) * omega * elapsed_time) / (2 * n + 1)
        servo_position = round(np.sum(fourier_series)/5) * 5 * direction + offset

        return servo_position

    # using a sinusoidal function to define back and forth servo movement
    def sin_servo_move(self, servo_position=0, amp=1000, omega=math.pi, offset=2048, direction=1):
        
        elapsed_time = time.time() - self.start_time

        servo_position = round(direction * amp * math.cos(omega * elapsed_time) + offset)

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