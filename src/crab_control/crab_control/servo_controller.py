# imports
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32MultiArray
import numpy as np
import time
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from crab_interfaces.msg import ServoData
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
            ServoData,
            '/servo/encoder_data',
            self.listener_callback,
            10)
        self.subscription

        # Time tracker
        self.start_time = time.time()

        self.position = 2048.0
        self.position2 = 2048
        self.counter = 0
<<<<<<< HEAD
        # Initial Servo positions
        self.servo_1_init = 1990
        self.servo_2_init = 1950
        self.servo_3_init = 1900
        self.servo_4_init = 1900
=======
>>>>>>> 2ef349f (rebase and merge off main. built but not run)


        self.latest_positions = [0, 0, 0, 0]

        # Variables for linear servo movement
        self.decreasing = True
        self.increasing = True


    # recieving encoder values and storing in class variable
    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % str(msg.data))
        self.latest_positions = list(msg.data)

    def timer_callback(self):

        # -----------------------------
        # Motion parameters
        # -----------------------------
        frequency = 0.5          # Hz

        roll_amplitude = 75      # degrees
        yaw_power = 85           # degrees during power stroke
        yaw_recovery = 5         # degrees during recovery

        power_fraction = 0.7    # portion of cycle spent pushing

        counts_per_degree = 4096 / 360

        roll_center = 2048
        yaw_center = 1650

        # Time since startup
        t = time.time() - self.start_time

        # Normalized cycle position (0 to 1)
        period = 1 / frequency
        u = (t % period) / period


        # -----------------------------
        # ROLL (Servo 1)
        # Sea turtle style:
        # slow power stroke
        # fast recovery stroke
        # -----------------------------
        if u < power_fraction:

            # Power stroke:
            # -amplitude -> +amplitude
            s = u / power_fraction

            roll_angle = (
                -roll_amplitude * math.cos(math.pi * s)
            )

        else:

            # Recovery stroke:
            # +amplitude -> -amplitude
            s = (u - power_fraction) / (1 - power_fraction)

            roll_angle = (
                roll_amplitude * math.cos(math.pi * s)
            )


        if u < power_fraction:
            yaw_angle = yaw_power
        else:
            yaw_angle = yaw_recovery

        # Convert degrees to encoder counts
        roll_position = int(
            roll_center +
            roll_angle * counts_per_degree
        )

        yaw_position = int(
            yaw_center +
            yaw_angle * counts_per_degree
        )


        # Keep positions safe
        roll_position = max(0, min(4095, roll_position))
        yaw_position = max(0, min(4095, yaw_position))


        # -----------------------------
        # Send commands
        # -----------------------------
        msg1 = SetPosition()
        msg1.id = 1
        msg1.position = roll_position


        msg2 = SetPosition()
        msg2.id = 2
        msg2.position = yaw_position


        self.publisher_.publish(msg1)
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