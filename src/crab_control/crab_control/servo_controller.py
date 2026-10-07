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

        # Initial Servo 
        self.servo_init = [2048, 2048, 2048, 2048] 
        self.servo_commands = [x for x in self.servo_init]
        self.latest_positions = [0, 0, 0, 0] # change for 4 servos

        # Initialization for april tag detection values
        self.tag_id = 0
        self.cx = 0
        self.cy = 0
        self.corners = [[0, 0, 0, 0], [0, 0, 0, 0]]
        self.size_x = 0
        self.size_y = 0
        self.size = 0


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
        # Video Width = 1280
        # Video Height = 720

        if self.tag_id == 1:
                # 1. Calculate how far the tag is from the center pixel (-640 to +640, -360 to +360)
                error_x = self.cx - 640
                error_y = self.cy - 360

                # 2. Convert pixel error to a small servo step (Tweak the 0.05/0.08 "gain" multipliers to change speed)
                # If tag is to the right (+X), yaw needs to turn right. If tag is down (+Y), roll needs to tilt down.
                yaw_step = round(error_x * 0.05)
                roll_step = round(error_y * 0.08)

                # 3. Nudge the CURRENT position instead of snapping to an absolute one
                # Assuming index 0 is Yaw and index 1 is Roll. Keeping indices 2 and 3 at 0 or unchanged.
                new_yaw = self.servo_commands[0] + yaw_step
                new_roll = self.servo_commands[1] + roll_step

                # 4. Constrain (clip) the values so they stay within the safe 0-4095 Dynamixel limits
                new_yaw = max(0, min(4095, new_yaw))
                new_roll = max(0, min(4095, new_roll))

                # Update your tracking array
                self.servo_commands = [2048, 2048, 2048, 2048]
        
        # Defining servo messages and IDs
        msg = SetPosition()
        # Publish the data
        for idx, command in enumerate(self.servo_commands):
            msg.id = idx + 1
            msg.position = command
            self.publisher_.publish(msg)

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