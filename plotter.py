#!/usr/bin/env python3

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import rosbag2_py
from rclpy.serialization import deserialize_message
from crab_interfaces.msg import LoadCell, ServoData

# ---------------- CONFIG ----------------

BAG_PATH = "/home/odinroast/crab_ws/adya_gait2.bag"

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

def read_bag():

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=BAG_PATH,
            storage_id="mcap"
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr")
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

# ---------------- Main ----------------

def main():

    load_t, load, cmd_t, cmd, enc_t, enc = read_bag()

    print(load.shape)
    print(cmd.shape)
    print(enc.shape)

    fig = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.07,
        subplot_titles=(
            "Load Cell Forces/Torques",
            "Servo Commands & Encoders"
        )
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
        title="ROS Bag Viewer",
        template="plotly_white",
        hovermode="x unified",
        height=900,
        legend=dict(
            orientation="h",
            y=1.02,
            x=0
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

    fig.write_html("rosbag_plot.html", auto_open=False)
    fig.show()


if __name__ == "__main__":
    main()