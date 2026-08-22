#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from crab_interfaces.msg import ServoData
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class FlipperJointPublisher(Node):
    def __init__(self):
        super().__init__('flipper_joint_publisher')

        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('roll', 0.0)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            ServoData,
            'servo/encoder_data',
            self.relay_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.trace_publisher = self.create_publisher(
            MarkerArray,
            '/flipper/traces',
            10
        )

        # Three point positions relative to flipper_link, in METERS.
        # Replace these with coordinates from your CAD model.
        self.flipper_points = [
            (0.207, 0.00, 0.00),   # point 1
            (0.207, 0.056, 0.00),   # point 2
            (0.207, -0.056, 0.00), # point 3
        ]

        self.trails = [[], [], []]
        self.trail_duration = 0.4  # seconds
        
    def dxl2js(self, pos):
        return ((pos/4096) * (2 * math.pi) - math.pi)

    def relay_callback(self, pos):
        # Array to store the angles
        angles = []

        # Servo 1 (YAW) Servo 2 (ROLL)
        for angle in pos.data:
            angles.append(self.dxl2js(angle))

        # Construct and publish joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = ['flipper_yaw_joint','flipper_roll_joint']

        msg.position = [
            angles[0],
            angles[1],
        ]

        self.publisher.publish(msg)
        self.publish_traces(angles[0], angles[1])
    
    def flipper_point_to_base(self, point, yaw, roll):
        x, y, z = point

        # Roll around X axis
        roll_y = math.cos(roll) * y - math.sin(roll) * z
        roll_z = math.sin(roll) * y + math.cos(roll) * z

        # Yaw around Z axis
        base_x = math.cos(yaw) * x - math.sin(yaw) * roll_y
        base_y = math.sin(yaw) * x + math.cos(yaw) * roll_y
        base_z = roll_z

        return Point(x=base_x, y=base_y, z=base_z)


    def publish_traces(self, yaw, roll):
        marker_array = MarkerArray()
        now_seconds = self.get_clock().now().nanoseconds / 1e9

        colors = [
            (1.0, 0.15, 0.15),  # red
            (0.15, 1.0, 0.25),  # green
            (1.0, 0.85, 0.1),   # yellow
        ]

        for i, local_point in enumerate(self.flipper_points):
            current = self.flipper_point_to_base(local_point, yaw, roll)

            # Add a point to its history only after it moves 2 mm.
            # Store point with its timestamp.
            if not self.trails[i] or math.dist(
                (current.x, current.y, current.z),
                (
                    self.trails[i][-1][0].x,
                    self.trails[i][-1][0].y,
                    self.trails[i][-1][0].z,
                )
            ) > 0.002:
                self.trails[i].append((current, now_seconds))

            # Keep only the most recent 0.5 seconds of movement.
            self.trails[i] = [
                (point, timestamp)
                for point, timestamp in self.trails[i]
                if now_seconds - timestamp <= self.trail_duration
            ]

            # Prevent endlessly growing trails.
            self.trails[i] = self.trails[i][-1000:]

            r, g, b = colors[i]

            # Current point: sphere
            sphere = Marker()
            sphere.header.frame_id = 'base_link'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'flipper_points'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = current
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.003
            sphere.scale.y = 0.003
            sphere.scale.z = 0.003
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            # Trail: line strip
            trail = Marker()
            trail.header.frame_id = 'base_link'
            trail.header.stamp = self.get_clock().now().to_msg()
            trail.ns = 'flipper_trails'
            trail.id = i
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.pose.orientation.w = 1.0
            trail.scale.x = 0.001
            trail.color.r = r
            trail.color.g = g
            trail.color.b = b
            trail.color.a = 1.0
            trail.points = trail.points = [point for point, _ in self.trails[i]]
            marker_array.markers.append(trail)

        self.trace_publisher.publish(marker_array)
        

def main():
    rclpy.init()
    node = FlipperJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()