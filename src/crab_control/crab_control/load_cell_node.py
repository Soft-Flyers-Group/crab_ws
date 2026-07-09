import socket
import numpy as np

import rclpy
from rclpy.node import Node

from crab_interfaces.msg import LoadCell


UDP_PORT = 5005

ROWS = 6
COLS = 20


class LoadCellPublisher(Node):

    def __init__(self):
        super().__init__('load_cell_publisher')

        self.publisher_ = self.create_publisher(
            LoadCell,
            '/load_cell_data',
            10
        )

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", UDP_PORT))

        self.get_logger().info(
            f"Listening for data on UDP port {UDP_PORT}"
        )

    def run(self):
        expected_size = ROWS * COLS * 4

        while rclpy.ok():

            data, _ = self.sock.recvfrom(65535)

            if len(data) != expected_size:
                self.get_logger().warning(
                    f"Bad packet size: {len(data)}"
                )
                continue

            # Decode UDP packet
            values = np.frombuffer(data, dtype=">f4")

            # Incoming: 6x20
            # Desired: 20x6
            matrix = values.reshape((ROWS, COLS)).T

            msg = LoadCell()

            # Timestamp when packet was received
            msg.header.stamp = self.get_clock().now().to_msg()

            # Optional frame name
            msg.header.frame_id = "load_cell"

            msg.rows = matrix.shape[0]   # 20
            msg.cols = matrix.shape[1]   # 6

            msg.data = matrix.flatten().tolist()

            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LoadCellPublisher()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()