#!/usr/bin/env python3
"""
inspect_msg.py

Prints the field names (and one sample message) for a topic in a rosbag2
mcap file, so you can see exactly what attributes are available.

Usage:
    python inspect_msg.py /path/to/bag_folder [topic]

Default topic: /load_cell_data
"""

import sys
import os
import yaml
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def main():
    bag_folder = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else "/load_cell_data"

    meta_path = os.path.join(bag_folder, "metadata.yaml")
    with open(meta_path, "r") as f:
        meta = yaml.safe_load(f)
    info = meta["rosbag2_bagfile_information"]
    mcap_path = os.path.join(bag_folder, info["relative_file_paths"][0])

    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[topic]
        ):
            print(f"Topic: {topic}")
            print(f"Type: {type(ros_msg)}")
            print(f"Fields: {[f for f in dir(ros_msg) if not f.startswith('_')]}")
            print("\nSample message:")
            print(ros_msg)
            break
        else:
            print(f"No messages found on topic {topic}")


if __name__ == "__main__":
    main()