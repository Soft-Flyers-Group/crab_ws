#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray


class TagOverlayNode(Node):
    def __init__(self):
        super().__init__("tag_overlay_node")

        self.latest_detections = []
        self.last_detection_time = 0.0
        self.detection_timeout = 0.5

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.detection_callback,
            10,
        )

        self.image_pub = self.create_publisher(
            Image,
            "/camera/image_tagged",
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Tag overlay started. Publishing on /camera/image_tagged"
        )

    def detection_callback(self, msg):
        self.latest_detections = list(msg.detections)
        self.last_detection_time = time.monotonic()

    @staticmethod
    def get_corner_pixels(detection):
        corners = []

        for corner in getattr(detection, "corners", []):
            if hasattr(corner, "x") and hasattr(corner, "y"):
                x, y = corner.x, corner.y
            else:
                x, y = corner[0], corner[1]

            corners.append((round(x), round(y)))

        return corners

    @staticmethod
    def draw_line(data, width, height, step, x0, y0, x1, y1):
        """Draw a green line directly into a bgr8/rgb8 image byte array."""

        def set_pixel(x, y):
            if 0 <= x < width and 0 <= y < height:
                index = y * step + x * 3
                data[index] = 0       # Blue
                data[index + 1] = 255 # Green
                data[index + 2] = 0   # Red

        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx - dy

        while True:
            set_pixel(x0, y0)

            if x0 == x1 and y0 == y1:
                break

            double_error = 2 * error

            if double_error > -dy:
                error -= dy
                x0 += sx

            if double_error < dx:
                error += dx
                y0 += sy

    def image_callback(self, msg):
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().error(
                f"Unsupported image encoding: {msg.encoding}. Expected bgr8 or rgb8."
            )
            return

        if msg.step < msg.width * 3 or len(msg.data) < msg.step * msg.height:
            self.get_logger().error("Camera image data has an invalid size.")
            return

        image_data = bytearray(msg.data)

        detections_are_fresh = (
            time.monotonic() - self.last_detection_time < self.detection_timeout
        )

        if detections_are_fresh:
            for detection in self.latest_detections:
                corners = self.get_corner_pixels(detection)

                if len(corners) < 4:
                    continue

                # Draw the tag's four sides.
                for i in range(len(corners)):
                    x0, y0 = corners[i]
                    x1, y1 = corners[(i + 1) % len(corners)]

                    self.draw_line(
                        image_data,
                        msg.width,
                        msg.height,
                        msg.step,
                        x0,
                        y0,
                        x1,
                        y1,
                    )

                # Draw an axis-aligned bounding box around the tag.
                x_min = min(point[0] for point in corners)
                x_max = max(point[0] for point in corners)
                y_min = min(point[1] for point in corners)
                y_max = max(point[1] for point in corners)

                self.draw_line(
                    image_data, msg.width, msg.height, msg.step,
                    x_min, y_min, x_max, y_min
                )
                self.draw_line(
                    image_data, msg.width, msg.height, msg.step,
                    x_max, y_min, x_max, y_max
                )
                self.draw_line(
                    image_data, msg.width, msg.height, msg.step,
                    x_max, y_max, x_min, y_max
                )
                self.draw_line(
                    image_data, msg.width, msg.height, msg.step,
                    x_min, y_max, x_min, y_min
                )

        output = Image()
        output.header = msg.header
        output.height = msg.height
        output.width = msg.width
        output.encoding = msg.encoding
        output.is_bigendian = msg.is_bigendian
        output.step = msg.step
        output.data = bytes(image_data)

        self.image_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TagOverlayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()