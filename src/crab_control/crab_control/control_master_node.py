import rclpy
from rclpy.node import Node
import math
import time
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction

GAIT_DURATION = 2.0       # seconds to commit to a gait before re-evaluating
DEADBAND      = 0.05      # if total error magnitude is below this, hover
TARGET_X = 0.0
TARGET_Y = 0.0
TARGET_Z = 0.15

# Force vectors [Fx, Fy, Fz] for each gait
GAIT_FORCE_VECTORS = {
    "gait_strafe_left":  [-1.8712, -0.562,   1.409 ],
    "gait_strafe_right": [ 1.8712, -0.562,   1.409 ],
    "gait_up":           [ 0.0,   -0.8204,   1.5562],
    "gait_forward":      [ 0.0,   -1.4611,  -0.2506],
}


def vector_magnitude(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def normalize(v):
    magnitude = vector_magnitude(v)
    if magnitude == 0:
        return [0.0, 0.0, 0.0]
    return [v[0] / magnitude, v[1] / magnitude, v[2] / magnitude]

def dot_product(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


class PositionController(Node):

    def __init__(self):
        super().__init__('position_controller')

        # TF listener for tag pose
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gait_publisher = self.create_publisher(String, '/gait_command', 10)

        self.current_gait     = "hover"
        self.last_switch_time = time.time()

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Position controller started")

    def get_error(self):
        try:
            # Look up where tag1 is relative to the camera
            transform = self.tf_buffer.lookup_transform(
                'camera_link',
                'tag1',
                rclpy.time.Time()
            )

            detected_x = transform.transform.translation.x
            detected_y = transform.transform.translation.y
            detected_z = transform.transform.translation.z

            error_x = TARGET_X - detected_x
            error_y = TARGET_Y - detected_y
            error_z = TARGET_Z - detected_z

            return [error_x, error_y, error_z]

        except Exception:
            # Tag not visible
            return None

    def select_gait(self, error_vec):
        error_magnitude = vector_magnitude(error_vec)

        # If within deadband, just hover
        if error_magnitude < DEADBAND:
            return "hover"

        # Normalize error into a unit direction
        error_direction = normalize(error_vec)

        # Score each gait by how well its thrust aligns with the error direction
        best_gait  = "hover"
        best_score = -float('inf')

        for gait_name, raw_force in GAIT_FORCE_VECTORS.items():
            # Auto-normalize so raw measurements go straight in
            gait_direction = normalize(raw_force)

            # Dot product: 1.0 = perfect alignment, -1.0 = pointing opposite way
            alignment_score = dot_product(gait_direction, error_direction)

            if alignment_score > best_score:
                best_score = alignment_score
                best_gait  = gait_name

        return best_gait

    def control_loop(self):
        now = time.time()

        error_vec = self.get_error()

        # If tag not visible, hover and wait
        if error_vec is None:
            if self.current_gait != "hover":
                self.get_logger().warn("Tag not detected — switching to hover")
                self.current_gait = "hover"
                self.publish_gait("hover")
            return

        # Only re-evaluate after the commitment window expires
        if now - self.last_switch_time < GAIT_DURATION:
            return

        # Select best gait for current error
        best_gait = self.select_gait(error_vec)

        # Switch if needed
        if best_gait != self.current_gait:
            self.get_logger().info(
                f"Switching gait: {self.current_gait} -> {best_gait} "
                f"(error: x={error_vec[0]:.3f}, y={error_vec[1]:.3f}, z={error_vec[2]:.3f})"
            )
            self.current_gait     = best_gait
            self.last_switch_time = now
            self.publish_gait(best_gait)

    def publish_gait(self, gait_name):
        msg = String()
        msg.data = gait_name
        self.gait_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()