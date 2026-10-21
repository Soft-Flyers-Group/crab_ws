import time

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from apriltag_msgs.msg import AprilTagDetectionArray


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.publisher_ = self.create_publisher(
            SetPosition, 'servo/set_position', 10
        )

        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        # Servo order: left yaw, left roll, right yaw, right roll.
        # Both yaw servos start forward; roll stays fixed.
        self.servo_commands = [3140, 2974, 660, 2924]

        # Your calibrated yaw limits.
        self.left_yaw_min = 1950       # Sideways
        self.left_yaw_max = 3140       # Forward: 1950 + 1190

        self.right_yaw_min = 660       # Forward: 1850 - 1190
        self.right_yaw_max = 1850      # Sideways

        # Camera and tracking settings.
        self.center_x = 640            # Half of 1280-pixel image width
        self.yaw_gain = 0.05
        self.yaw_direction = 1         # Change to -1 to reverse tracking
        self.deadband = 10             # Ignore small pixel errors

        self.target_id = 1
        self.tag_visible = False
        self.cx = self.center_x
        self.last_tag_time = None
        self.tag_timeout = 0.5         # Hold position if detection goes stale

        self.timer = self.create_timer(0.02, self.timer_callback)

    def tag_callback(self, msg):
        # Look for tag 1, even if other tags appear before it.
        self.tag_visible = False

        for detection in msg.detections:
            if detection.id == self.target_id:
                self.cx = detection.centre.x
                self.last_tag_time = time.monotonic()
                self.tag_visible = True
                break

    def timer_callback(self):
        tag_is_fresh = (
            self.tag_visible
            and self.last_tag_time is not None
            and time.monotonic() - self.last_tag_time < self.tag_timeout
        )

        if tag_is_fresh:
            error_x = (self.cx - self.center_x) * self.yaw_direction

            # Tag near the center: both flippers point forward.
            if abs(error_x) <= self.deadband:
                error_x = 0

            # Map image position to yaw:
            # center = 0, left edge = -1, right edge = +1.
            offset = max(-1.0, min(1.0, error_x / self.center_x))

            # Left tag: left flipper turns from forward toward sideways.
            # Right tag: right flipper turns from forward toward sideways.
            self.servo_commands[0] = round(
                self.left_yaw_max
                - max(0.0, -offset)
                * (self.left_yaw_max - self.left_yaw_min)
            )

            self.servo_commands[2] = round(
                self.right_yaw_min
                + max(0.0, offset)
                * (self.right_yaw_max - self.right_yaw_min)
            )

        # Keep roll fixed.
        self.servo_commands[1] = 2974
        self.servo_commands[3] = 2924

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