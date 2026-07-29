import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32MultiArray, String
import numpy as np
import time
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from crab_interfaces.msg import ServoData


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('servo_controller')
        self.publisher_ = self.create_publisher(SetPosition, 'servo/set_position', 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            ServoData,
            '/servo/encoder_data',
            self.listener_callback,
            10)

        self.gait_subscription = self.create_subscription(
            String,
            '/gait_command',
            self.gait_callback,
            10)

        self.start_time = time.time()
        self.position = 2048.0
        self.position2 = 2048
        self.counter = 0
        self.latest_positions = [0, 0]
        self.decreasing = True
        self.increasing = True
        self.current_gait = "hover"

    def listener_callback(self, msg):
        self.latest_positions = list(msg.data)

    def gait_callback(self, msg):
        self.current_gait = msg.data
        self.get_logger().info(f"Switching to gait: {self.current_gait}")

    # -----------------------------
    # Gait functions
    # each returns: roll_a, yaw_a, roll_b, yaw_b, roll_center, yaw_center
    # A = flipper IDs 1,2 — B = flipper IDs 3,4
    # -----------------------------

    def gait_up(self):
        # power_fraction=0.7, yaw sinusoidal — primary force: UP
        frequency = 0.5
        roll_amplitude = 75
        yaw_amplitude = 45
        roll_center = 2048
        yaw_center = 1000
        power_fraction = 0.7

        t = time.time() - self.start_time
        u = (t % (1 / frequency)) / (1 / frequency)

        if u < power_fraction:
            s = u / power_fraction
            roll_angle = -roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = -yaw_amplitude  * math.sin(math.pi * s)
        else:
            s = (u - power_fraction) / (1 - power_fraction)
            roll_angle = roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = yaw_amplitude  * math.sin(math.pi * s)

        # Both flippers run identically
        return roll_angle, yaw_angle, roll_angle, -yaw_angle, roll_center, yaw_center

    def gait_forward(self):
        # power_fraction=0.7, yaw flat — primary force: FORWARD
        frequency = 0.5
        roll_amplitude = 75
        yaw_power    = 90
        yaw_recovery = 5
        roll_center = 2048
        yaw_center = 1000
        power_fraction = 0.7

        t = time.time() - self.start_time
        u = (t % (1 / frequency)) / (1 / frequency)

        if u < power_fraction:
            s = u / power_fraction
            roll_angle = -roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = yaw_power
        else:
            s = (u - power_fraction) / (1 - power_fraction)
            roll_angle = roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = yaw_recovery

        # Both flippers run identically
        return roll_angle, yaw_angle, roll_angle, yaw_angle, roll_center, yaw_center

    def gait_strafe_left(self):
        # power_fraction=0.5, yaw sinusoidal — primary force: SIDE
        # Only flipper A (IDs 1,2) runs, flipper B (IDs 3,4) holds center
        frequency = 0.5
        roll_amplitude = 75
        yaw_amplitude = 45
        roll_center = 2048
        yaw_center = 1000
        power_fraction = 0.5

        t = time.time() - self.start_time
        u = (t % (1 / frequency)) / (1 / frequency)

        if u < power_fraction:
            s = u / power_fraction
            roll_angle = -roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = -yaw_amplitude  * math.sin(math.pi * s)
        else:
            s = (u - power_fraction) / (1 - power_fraction)
            roll_angle = roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = yaw_amplitude  * math.sin(math.pi * s)

        # Flipper A moves, flipper B holds center (angle=0)
        return roll_angle, yaw_angle, 0, 0, roll_center, yaw_center

    def gait_strafe_right(self):
        # power_fraction=0.5, yaw sinusoidal — primary force: SIDE
        # Only flipper B (IDs 3,4) runs, flipper A (IDs 1,2) holds center
        frequency = 0.5
        roll_amplitude = 75
        yaw_amplitude = 45
        roll_center = 2048
        yaw_center = 1000
        power_fraction = 0.5

        t = time.time() - self.start_time
        u = (t % (1 / frequency)) / (1 / frequency)

        if u < power_fraction:
            s = u / power_fraction
            roll_angle = -roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = -yaw_amplitude  * math.sin(math.pi * s)
        else:
            s = (u - power_fraction) / (1 - power_fraction)
            roll_angle = roll_amplitude * math.cos(math.pi * s)
            yaw_angle  = yaw_amplitude  * math.sin(math.pi * s)

        # Flipper B moves, flipper A holds center (angle=0)
        return 0, 0, roll_angle, yaw_angle, roll_center, yaw_center

    def gait_hover(self):
        # Both flippers hold center
        return 0, 0, 0, 0, 2048, 1000

    # -----------------------------
    # Timer callback — runs at 50Hz
    # -----------------------------

    def timer_callback(self):
        counts_per_degree = 4096 / 360

        if self.current_gait == "gait_up":
            roll_a, yaw_a, roll_b, yaw_b, roll_center, yaw_center = self.gait_up()
        elif self.current_gait == "gait_forward":
            roll_a, yaw_a, roll_b, yaw_b, roll_center, yaw_center = self.gait_forward()
        elif self.current_gait == "gait_strafe_left":
            roll_a, yaw_a, roll_b, yaw_b, roll_center, yaw_center = self.gait_strafe_left()
        elif self.current_gait == "gait_strafe_right":
            roll_a, yaw_a, roll_b, yaw_b, roll_center, yaw_center = self.gait_strafe_right()
        else:  # hover
            roll_a, yaw_a, roll_b, yaw_b, roll_center, yaw_center = self.gait_hover()

        # Convert to encoder counts
        pos_1 = int(roll_center + roll_a * counts_per_degree)
        pos_2 = int(yaw_center  + yaw_a  * counts_per_degree)
        pos_3 = int(roll_center + roll_b * counts_per_degree)
        pos_4 = int(yaw_center  + yaw_b  * counts_per_degree)

        # Clamp to safe range
        pos_1 = max(0, min(4095, pos_1))
        pos_2 = max(0, min(4095, pos_2))
        pos_3 = max(0, min(4095, pos_3))
        pos_4 = max(0, min(4095, pos_4))

        # Publish all 4 servos
        for servo_id, position in [(1, pos_1), (2, pos_2), (3, pos_3), (4, pos_4)]:
            msg = SetPosition()
            msg.id = servo_id
            msg.position = position
            self.publisher_.publish(msg)


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
