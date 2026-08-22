#!/usr/bin/env python3

"""Interactive ROS 2 bag viewer grouped by flipper type."""

from functools import lru_cache
from pathlib import Path
import re

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dash import Dash, Input, Output, State, dcc, html

import rosbag2_py
from rclpy.serialization import deserialize_message
from crab_interfaces.msg import LoadCell, ServoData


BAG_DIR = Path("/home/odinroast/crab_ws/bags")

FLIPPER_TYPES = (
    "control",
    "fiberglass",
    "petg_thin",
    "rib_flipper",
    "stripe_flipper",
)

TOPICS = {
    "/load_cell_data": LoadCell,
    "/servo/position_data": ServoData,
    "/servo/encoder_data": ServoData,
}

LOAD_LABELS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
LOAD_COLORS = ("red", "green", "blue", "orange", "purple", "black")
SERVO_COUNT = 2  # Each bag contains data for one two-servo flipper.
SERVO_COLORS = (
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
)


def natural_key(text):
    """Sort test 2 before test 10."""
    return [int(part) if part.isdigit() else part.lower()
            for part in re.split(r"(\d+)", str(text))]


def discover_bags():
    """Return {flipper_type: [bag paths]} for files currently on disk."""
    groups = {name: [] for name in FLIPPER_TYPES}

    for path in BAG_DIR.glob("*.bag"):
        name = path.name.lower()
        for flipper in FLIPPER_TYPES:
            if name.startswith(flipper.lower()):
                groups[flipper].append(path)
                break

    for paths in groups.values():
        paths.sort(key=lambda path: natural_key(path.name))

    return groups


def display_test_name(path, flipper):
    """Make the bag dropdown shorter while retaining the full test name."""
    name = path.stem
    prefix = f"{flipper}_"
    suffix = f"_{flipper}"

    if name.lower().startswith(prefix.lower()):
        name = name[len(prefix):]
    if name.lower().endswith(suffix.lower()):
        name = name[:-len(suffix)]
    return name


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def flatten_load_batches(batch_times, batches):
    """Expand batched (rows x 6) readings into one continuous time series.

    Rows inside each message are treated as consecutive samples. Their times are
    evenly spaced from that message's stamp up to (but not including) the next
    message's stamp. For the final message, the median preceding message period
    is used.
    """
    if not batches:
        return np.empty(0, dtype=np.float64), np.empty((0, 6), dtype=np.float32)

    times = np.asarray(batch_times, dtype=np.float64)
    periods = np.diff(times)
    positive_periods = periods[periods > 0]
    fallback_period = float(np.median(positive_periods)) if positive_periods.size else 0.0

    sample_times = []
    sample_values = []
    for i, batch in enumerate(batches):
        row_count = batch.shape[0]
        period = times[i + 1] - times[i] if i + 1 < len(times) else fallback_period

        if period > 0:
            row_times = times[i] + np.arange(row_count, dtype=np.float64) * (period / row_count)
        else:
            row_times = np.full(row_count, times[i], dtype=np.float64)

        sample_times.append(row_times)
        sample_values.append(batch)

    return np.concatenate(sample_times), np.concatenate(sample_values, axis=0)


@lru_cache(maxsize=4)
def read_bag(bag_path):
    """Read one selected bag. The four most recent bags stay cached."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    data = {
        "load_t": [], "load": [],
        "cmd_t": [], "cmd": [],
        "enc_t": [], "enc": [],
    }

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        message_type = TOPICS.get(topic)
        if message_type is None:
            continue

        msg = deserialize_message(raw, message_type)
        t = stamp_to_sec(msg.header.stamp)

        if topic == "/load_cell_data":
            matrix = np.asarray(msg.data, dtype=np.float32)
            expected = msg.rows * msg.cols
            if matrix.size != expected:
                raise ValueError(
                    f"LoadCell contains {matrix.size} values, but rows*cols "
                    f"is {msg.rows}*{msg.cols}={expected}."
                )
            data["load_t"].append(t)
            data["load"].append(matrix.reshape(msg.rows, msg.cols))
        elif topic == "/servo/position_data":
            data["cmd_t"].append(t)
            data["cmd"].append(np.asarray(msg.data, dtype=np.float32))
        elif topic == "/servo/encoder_data":
            data["enc_t"].append(t)
            data["enc"].append(np.asarray(msg.data, dtype=np.float32))

    load_t, load = flatten_load_batches(data["load_t"], data["load"])
    timestamps = load_t.tolist() + data["cmd_t"] + data["enc_t"]
    time_zero = min(timestamps) if timestamps else 0.0

    def times(key):
        return np.asarray(data[key], dtype=np.float64) - time_zero

    return {
        "load_t": load_t - time_zero,
        "load": load,
        "cmd_t": times("cmd_t"),
        "cmd": np.asarray(data["cmd"]),
        "enc_t": times("enc_t"),
        "enc": np.asarray(data["enc"]),
    }


def empty_figure(message="Select a bag to begin"):
    fig = go.Figure()
    fig.add_annotation(text=message, x=0.5, y=0.5, showarrow=False)
    fig.update_layout(template="plotly_white", height=850)
    return fig


def make_figure(data, bag_name):
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

    load = data["load"]
    if load.size and load.ndim == 2:
        wrench = load
        for i in range(min(6, wrench.shape[1])):
            fig.add_trace(
                go.Scattergl(
                    x=data["load_t"], y=wrench[:, i], mode="lines",
                    name=LOAD_LABELS[i],
                    line=dict(color=LOAD_COLORS[i], width=2),
                ),
                row=1, col=1,
            )

    for key, time_key, suffix, dash in (
        ("cmd", "cmd_t", "Cmd", "solid"),
        ("enc", "enc_t", "Enc", "dash"),
    ):
        values = data[key]
        if values.size and values.ndim == 2:
            for i in range(min(SERVO_COUNT, values.shape[1])):
                color = SERVO_COLORS[i % len(SERVO_COLORS)]
                fig.add_trace(
                    go.Scattergl(
                        x=data[time_key], y=values[:, i], mode="lines",
                        name=f"S{i + 1} {suffix}",
                        line=dict(color=color, dash=dash),
                    ),
                    row=2, col=1,
                )

    fig.update_layout(
        title=bag_name,
        template="plotly_white",
        hovermode="x unified",
        height=900,
        legend=dict(orientation="h", y=1.08, x=0),
        margin=dict(l=70, r=30, t=135, b=60),
    )
    fig.update_xaxes(title_text="Time from bag start (s)", row=2, col=1)
    fig.update_yaxes(title_text="Force / Torque", row=1, col=1)
    fig.update_yaxes(title_text="Servo Position", row=2, col=1)
    return fig


app = Dash(__name__)
app.title = "CRAB Rosbag Viewer"

CONTROL_STYLE = {"minWidth": "260px", "flex": "1"}

app.layout = html.Div(
    [
        html.H1("CRAB Rosbag Viewer", style={"marginBottom": "4px"}),
        html.P(
            f"Reading MCAP bags from {BAG_DIR}",
            style={"color": "#555", "marginTop": "0"},
        ),
        html.Div(
            [
                html.Div(
                    [html.Label("Flipper type"), dcc.Dropdown(
                        id="flipper-dropdown",
                        options=[{"label": x, "value": x} for x in FLIPPER_TYPES],
                        value="control",
                        clearable=False,
                    )],
                    style=CONTROL_STYLE,
                ),
                html.Div(
                    [html.Label("Test / bag"), dcc.Dropdown(
                        id="bag-dropdown", clearable=False,
                    )],
                    style={**CONTROL_STYLE, "flex": "2"},
                ),
                html.Button("Refresh bag list", id="refresh-button", n_clicks=0,
                            style={"height": "38px", "alignSelf": "end"}),
            ],
            style={"display": "flex", "gap": "14px", "flexWrap": "wrap"},
        ),
        dcc.Loading(
            [html.Div(id="status", style={"margin": "12px 0"}),
             dcc.Graph(id="bag-graph", figure=empty_figure())],
            type="circle",
        ),
    ],
    style={"maxWidth": "1500px", "margin": "0 auto", "padding": "18px"},
)


@app.callback(
    Output("bag-dropdown", "options"),
    Output("bag-dropdown", "value"),
    Input("flipper-dropdown", "value"),
    Input("refresh-button", "n_clicks"),
)
def update_bag_options(flipper, _refresh_clicks):
    paths = discover_bags().get(flipper, [])
    options = [
        {"label": display_test_name(path, flipper), "value": str(path)}
        for path in paths
    ]
    return options, (options[0]["value"] if options else None)


@app.callback(
    Output("bag-graph", "figure"),
    Output("status", "children"),
    Input("bag-dropdown", "value"),
    State("flipper-dropdown", "value"),
)
def update_graph(bag_path, flipper):
    if not bag_path:
        return empty_figure(f"No .bag files found for {flipper}"), \
            f"No bags found for flipper type: {flipper}"

    try:
        data = read_bag(bag_path)
        figure = make_figure(data, Path(bag_path).name)
        status = (
            f"{Path(bag_path).name} | "
            f"load samples: {len(data['load_t']):,} | "
            f"command samples: {len(data['cmd_t']):,} | "
            f"encoder samples: {len(data['enc_t']):,}"
        )
        return figure, status
    except Exception as exc:
        message = f"Could not read {Path(bag_path).name}: {exc}"
        return empty_figure(message), message


if __name__ == "__main__":
    if not BAG_DIR.is_dir():
        raise SystemExit(f"Bag directory does not exist: {BAG_DIR}")
    app.run(debug=False, host="127.0.0.1", port=8050)