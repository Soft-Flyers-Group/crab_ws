#!/usr/bin/env python3

from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import rosbag2_py
from rclpy.serialization import deserialize_message
from crab_interfaces.msg import LoadCell, ServoData

# ---------------- CONFIG ----------------
<<<<<<< HEAD
# Change to your usename
BAG_PATH = "/home/odinroast/crab_ws/adya_gait2.bag"
=======

<<<<<<< HEAD
BAG_PATH = "/home/vmookim/crab_ws/experiment_01.bag"
>>>>>>> d261325 (refined code after rebase with data collected)
=======
BAG_DIR = Path("/home/vmookim/crab_ws")
>>>>>>> a5d4e2a (working code)

TOPICS = {
    "/load_cell_data": LoadCell,
    "/servo/position_data": ServoData,
    "/servo/encoder_data": ServoData,
}

# ---------------- Helpers ----------------


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def normalize_time(t):
    return t - t[0] if len(t) else t


# ---------------- Read bag ----------------


def read_bag(bag_path):

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id="mcap",
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    data = {
        "load_t": [],
        "load": [],
        "cmd_t": [],
        "cmd": [],
        "enc_t": [],
        "enc": [],
    }

    while reader.has_next():

        topic, raw, _ = reader.read_next()

        if topic not in TOPICS:
            continue

        msg = deserialize_message(raw, TOPICS[topic])

        t = stamp_to_sec(msg.header.stamp)

        if topic == "/load_cell_data":

            mat = np.array(msg.data, dtype=np.float32)
            mat = mat.reshape(msg.rows, msg.cols)

            data["load_t"].append(t)
            data["load"].append(mat)

        elif topic == "/servo/position_data":

            data["cmd_t"].append(t)
            data["cmd"].append(msg.data)

        elif topic == "/servo/encoder_data":

            data["enc_t"].append(t)
            data["enc"].append(msg.data)

    return (
        normalize_time(np.array(data["load_t"])),
        np.array(data["load"]),
        normalize_time(np.array(data["cmd_t"])),
        np.array(data["cmd"]),
        normalize_time(np.array(data["enc_t"])),
        np.array(data["enc"]),
    )


# ---------------- Plot one bag ----------------


def plot_bag(bag_path):

    load_t, load, cmd_t, cmd, enc_t, enc = read_bag(bag_path)

    print(f"\nProcessing {bag_path.name}")
    print("Load:", load.shape)
    print("Cmd :", cmd.shape)
    print("Enc :", enc.shape)

    fig = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.07,
        subplot_titles=(
            "Load Cell Forces/Torques",
            "Servo Commands & Encoders",
        ),
    )

    # ---------- Load Cell ----------

    if len(load):

        wrench = load[:, 0, :]

        labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
        colors = ["red", "green", "blue", "orange", "purple", "black"]

        for i in range(6):

            fig.add_trace(
                go.Scattergl(
                    x=load_t,
                    y=wrench[:, i],
                    mode="lines",
                    name=labels[i],
                    line=dict(color=colors[i], width=2),
                ),
                row=1,
                col=1,
            )

    # ---------- Servo Data ----------

    servo_colors = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
        "#7f7f7f",
    ]

    if len(cmd):

        n = cmd.shape[1]

        for i in range(n):

            c = servo_colors[i % len(servo_colors)]

            fig.add_trace(
                go.Scattergl(
                    x=cmd_t,
                    y=cmd[:, i],
                    mode="lines",
                    name=f"S{i+1} Cmd",
                    line=dict(color=c),
                ),
                row=2,
                col=1,
            )

    if len(enc):

        n = enc.shape[1]

        for i in range(n):

            c = servo_colors[i % len(servo_colors)]

            fig.add_trace(
                go.Scattergl(
                    x=enc_t,
                    y=enc[:, i],
                    mode="lines",
                    name=f"S{i+1} Enc",
                    line=dict(color=c, dash="dash"),
                ),
                row=2,
                col=1,
            )

    # ---------- Layout ----------

    fig.update_layout(
        title=f"ROS Bag Viewer - {bag_path.name}",
        template="plotly_white",
        hovermode="x unified",
        height=900,
        legend=dict(
            orientation="h",
            y=1.02,
            x=0,
        ),
    )

    fig.update_xaxes(
        title="Time (s)",
        rangeslider_visible=True,
        row=2,
        col=1,
    )

    fig.update_yaxes(title="Force / Torque", row=1, col=1)
    fig.update_yaxes(title="Servo Position", row=2, col=1)

    out_file = bag_path / "rosbag_plot.html"
    fig.write_html(str(out_file), auto_open=False)

    print(f"Saved {out_file}")


# ---------------- Main ----------------


def main():

    bag_paths = sorted(BAG_DIR.glob("*.bag"))

    if not bag_paths:
        print(f"No .bag directories found in {BAG_DIR}")
        return

    print(f"Found {len(bag_paths)} bag(s)")

    for bag in bag_paths:
        try:
            plot_bag(bag)
        except Exception as e:
            print(f"Failed to process {bag.name}: {e}")


if __name__ == "__main__":
    main()