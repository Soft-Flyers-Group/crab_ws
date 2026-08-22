#!/usr/bin/env python3

"""Create a graph-heavy PowerPoint containing every CRAB flipper bag/run.

Deck order:
    overview -> flipper -> gait -> run

Each run gets a full-screen 16:9 graph slide containing:
    1. Flattened load-cell Fx/Fy/Fz/Tx/Ty/Tz
    2. Servo 1 and Servo 2 commands/encoders only
"""

import argparse
from collections import defaultdict
from pathlib import Path
import re
from tempfile import TemporaryDirectory

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.util import Inches, Pt

import rosbag2_py
from rclpy.serialization import deserialize_message
from crab_interfaces.msg import LoadCell, ServoData


FLIPPERS = ("control", "fiberglass", "petg_thin", "rib_flipper", "stripe_flipper")
TOPICS = {
    "/load_cell_data": LoadCell,
    "/servo/position_data": ServoData,
    "/servo/encoder_data": ServoData,
}
LOAD_LABELS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
LOAD_COLORS = ("#d62728", "#2ca02c", "#1f77b4", "#ff7f0e", "#9467bd", "#111111")
SERVO_COLORS = ("#1f77b4", "#ff7f0e")
MAX_PLOT_POINTS = 15_000

SLIDE_W = Inches(13.333333)
SLIDE_H = Inches(7.5)
NAVY = RGBColor(18, 31, 53)
BLUE = RGBColor(61, 141, 255)
LIGHT_BLUE = RGBColor(208, 237, 250)
WHITE = RGBColor(255, 255, 255)
GRAY = RGBColor(92, 101, 114)


def natural_key(value):
    return [int(part) if part.isdigit() else part.lower()
            for part in re.split(r"(\d+)", str(value))]


def identify_flipper(filename):
    lower = filename.lower()
    for flipper in FLIPPERS:
        if lower.startswith(flipper.lower()):
            return flipper
    return None


def parse_gait_and_run(path, flipper):
    stem = path.stem
    if flipper == "fiberglass":
        experiment = stem[len("fiberglass"):].lstrip("_")
        match = re.match(r"^(.*?)(\d+)$", experiment)
        return ((match.group(1), int(match.group(2))) if match
                else (experiment or "unknown", 1))

    prefix = f"{flipper}_"
    suffix = f"_{flipper}"
    experiment = stem[len(prefix):] if stem.lower().startswith(prefix.lower()) else stem
    if experiment.lower().endswith(suffix.lower()):
        experiment = experiment[:-len(suffix)]
    if experiment.lower().startswith("vansh_"):
        experiment = experiment[len("vansh_"):]
    match = re.match(r"^(.*)_(\d+)$", experiment)
    return ((match.group(1), int(match.group(2))) if match
            else (experiment or "unknown", 1))


def friendly(value):
    names = {
        "control": "Control",
        "fiberglass": "Fiberglass",
        "petg_thin": "PETG Thin",
        "rib_flipper": "Rib Flipper",
        "stripe_flipper": "Stripe Flipper",
        "DOUBLESIN": "Double Sin",
        "experimental_sinusoidalYaw": "Experimental Sinusoidal Yaw",
        "FOURIER": "Fourier",
        "NESTEDSIN": "Nested Sin",
        "rollAmpAndFrequency": "Roll Amplitude and Frequency",
        "SIN": "Sin",
        "SINFOURIER": "Sin + Fourier",
        "SINmiddle": "Sin Middle",
        "SLOWFAST": "Slow / Fast",
        "yawPower": "Yaw Power",
    }
    return names.get(value, value)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def flatten_load_batches(batch_times, batches):
    if not batches:
        return np.empty(0), np.empty((0, 6), dtype=np.float32)

    times = np.asarray(batch_times, dtype=np.float64)
    positive_periods = np.diff(times)
    positive_periods = positive_periods[positive_periods > 0]
    fallback = float(np.median(positive_periods)) if positive_periods.size else 0.0
    expanded_times, expanded_values = [], []

    for index, batch in enumerate(batches):
        rows = batch.shape[0]
        period = times[index + 1] - times[index] if index + 1 < len(times) else fallback
        if period > 0:
            row_times = times[index] + np.arange(rows) * (period / rows)
        else:
            row_times = np.full(rows, times[index])
        expanded_times.append(row_times)
        expanded_values.append(batch)

    return np.concatenate(expanded_times), np.concatenate(expanded_values, axis=0)


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    values = {"load_t": [], "load": [], "cmd_t": [], "cmd": [], "enc_t": [], "enc": []}

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        message_type = TOPICS.get(topic)
        if message_type is None:
            continue
        msg = deserialize_message(raw, message_type)
        time = stamp_to_sec(msg.header.stamp)

        if topic == "/load_cell_data":
            matrix = np.asarray(msg.data, dtype=np.float32)
            expected = msg.rows * msg.cols
            if matrix.size != expected:
                raise ValueError(f"LoadCell has {matrix.size} values; expected {expected}")
            values["load_t"].append(time)
            values["load"].append(matrix.reshape(msg.rows, msg.cols))
        elif topic == "/servo/position_data":
            values["cmd_t"].append(time)
            values["cmd"].append(np.asarray(msg.data, dtype=np.float32))
        elif topic == "/servo/encoder_data":
            values["enc_t"].append(time)
            values["enc"].append(np.asarray(msg.data, dtype=np.float32))

    load_t, load = flatten_load_batches(values["load_t"], values["load"])
    all_times = load_t.tolist() + values["cmd_t"] + values["enc_t"]
    time_zero = min(all_times) if all_times else 0.0

    return {
        "load_t": load_t - time_zero,
        "load": load,
        "cmd_t": np.asarray(values["cmd_t"]) - time_zero,
        "cmd": np.asarray(values["cmd"]),
        "enc_t": np.asarray(values["enc_t"]) - time_zero,
        "enc": np.asarray(values["enc"]),
    }


def downsample(time, values, maximum=MAX_PLOT_POINTS):
    if len(time) <= maximum:
        return time, values
    step = int(np.ceil(len(time) / maximum))
    return time[::step], values[::step]


def save_run_graph(data, path, flipper, gait, run):
    fig, axes = plt.subplots(
        2, 1, figsize=(16, 9), dpi=160, sharex=True,
        gridspec_kw={"height_ratios": [1.12, 1], "hspace": 0.14},
    )
    fig.patch.set_facecolor("white")
    fig.suptitle(
        f"{friendly(flipper)}  |  {friendly(gait)}  |  Run {run}",
        x=0.055, y=0.975, ha="left", fontsize=22, fontweight="bold", color="#121F35",
    )
    fig.text(0.945, 0.972, path.name, ha="right", va="top", fontsize=9, color="#68707D")

    load = data["load"]
    if load.size and load.ndim == 2:
        t, plotted = downsample(data["load_t"], load)
        for index in range(min(6, plotted.shape[1])):
            axes[0].plot(t, plotted[:, index], label=LOAD_LABELS[index],
                         color=LOAD_COLORS[index], linewidth=1.25)
    else:
        axes[0].text(0.5, 0.5, "No load-cell data", transform=axes[0].transAxes,
                     ha="center", va="center", fontsize=18, color="#68707D")

    for key, time_key, suffix, linestyle in (
        ("cmd", "cmd_t", "Command", "-"),
        ("enc", "enc_t", "Encoder", "--"),
    ):
        servo = data[key]
        if servo.size and servo.ndim == 2:
            t, plotted = downsample(data[time_key], servo)
            for index in range(min(2, plotted.shape[1])):
                axes[1].plot(t, plotted[:, index],
                             label=f"Servo {index + 1} {suffix}",
                             color=SERVO_COLORS[index], linestyle=linestyle,
                             linewidth=1.35)

    axes[0].set_title("Load-cell forces and torques", loc="left", fontsize=13, fontweight="bold")
    axes[0].set_ylabel("Force / Torque")
    axes[1].set_title("Servo 1–2 commands and encoders", loc="left", fontsize=13, fontweight="bold")
    axes[1].set_ylabel("Servo Position")
    axes[1].set_xlabel("Time from bag start (s)")

    for axis in axes:
        axis.grid(True, color="#D9DDE3", linewidth=0.65, alpha=0.9)
        axis.spines[["top", "right"]].set_visible(False)
        axis.spines[["left", "bottom"]].set_color("#AEB5BF")
        axis.legend(loc="upper right", ncol=6, fontsize=9, frameon=False)
        axis.margins(x=0)

    fig.subplots_adjust(left=0.065, right=0.97, top=0.90, bottom=0.075)
    fig.savefig(path.with_suffix(".png"), facecolor="white")
    plt.close(fig)
    return path.with_suffix(".png")


def set_background(slide, color):
    fill = slide.background.fill
    fill.solid()
    fill.fore_color.rgb = color


def add_text(slide, text, left, top, width, height, size, color, bold=False,
             align=PP_ALIGN.LEFT):
    box = slide.shapes.add_textbox(Inches(left), Inches(top), Inches(width), Inches(height))
    frame = box.text_frame
    frame.clear()
    frame.margin_left = frame.margin_right = 0
    frame.margin_top = frame.margin_bottom = 0
    paragraph = frame.paragraphs[0]
    paragraph.text = text
    paragraph.alignment = align
    run = paragraph.runs[0]
    run.font.name = "Arial"
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = color
    return box


def add_title_slide(prs, total_bags, group_count):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_background(slide, NAVY)
    add_text(slide, "CRAB FLIPPER TESTING", 0.65, 0.45, 5.5, 0.35, 15, LIGHT_BLUE, True)
    add_text(slide, "Flipper gait and run graphs", 0.65, 2.15, 11.8, 0.85, 38, WHITE, True)
    add_text(slide, f"{total_bags} ROS bags • {len(FLIPPERS)} flipper types • {group_count} gait families",
             0.65, 3.15, 11.8, 0.45, 20, WHITE)
    add_text(slide, "Each run shows flattened load-cell data and Servo 1–2 tracking.",
             0.65, 5.95, 11.8, 0.4, 17, LIGHT_BLUE)


def add_section_slide(prs, label, eyebrow, detail):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_background(slide, WHITE)
    add_text(slide, eyebrow.upper(), 0.65, 0.5, 8.5, 0.35, 14, BLUE, True)
    add_text(slide, label, 0.65, 2.2, 11.8, 0.85, 38, NAVY, True)
    add_text(slide, detail, 0.65, 3.25, 11.8, 0.5, 19, GRAY)
    line = slide.shapes.add_shape(1, Inches(0.65), Inches(5.95), Inches(12.0), Inches(0.08))
    line.fill.solid()
    line.fill.fore_color.rgb = LIGHT_BLUE
    line.line.fill.background()


def add_graph_slide(prs, image_path):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    slide.shapes.add_picture(str(image_path), 0, 0, width=SLIDE_W, height=SLIDE_H)


def discover_groups(bag_dir):
    groups = defaultdict(lambda: defaultdict(list))
    for path in bag_dir.glob("*.bag"):
        flipper = identify_flipper(path.name)
        if flipper is None:
            continue
        gait, run = parse_gait_and_run(path, flipper)
        groups[flipper][gait].append((run, path))
    for gait_map in groups.values():
        for entries in gait_map.values():
            entries.sort(key=lambda item: (item[0], natural_key(item[1].name)))
    return groups


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-dir", type=Path,
                        default=Path("/home/odinroast/crab_ws/bags"))
    parser.add_argument("--output", type=Path,
                        default=Path("flipper_all_run_graphs.pptx"))
    parser.add_argument("--keep-plots", type=Path,
                        help="Optional directory in which generated PNG graphs are retained")
    args = parser.parse_args()

    if not args.bag_dir.is_dir():
        raise SystemExit(f"Bag directory does not exist: {args.bag_dir}")

    groups = discover_groups(args.bag_dir)
    total_bags = sum(len(entries) for gait_map in groups.values() for entries in gait_map.values())
    gait_count = sum(len(gait_map) for gait_map in groups.values())
    if total_bags == 0:
        raise SystemExit(f"No recognized .bag files found in {args.bag_dir}")

    prs = Presentation()
    prs.slide_width = SLIDE_W
    prs.slide_height = SLIDE_H
    add_title_slide(prs, total_bags, gait_count)

    errors = []
    temporary = None
    if args.keep_plots:
        plot_dir = args.keep_plots
        plot_dir.mkdir(parents=True, exist_ok=True)
    else:
        temporary = TemporaryDirectory(prefix="crab_pptx_plots_")
        plot_dir = Path(temporary.name)

    try:
        for flipper in FLIPPERS:
            gait_map = groups.get(flipper, {})
            if not gait_map:
                continue
            flipper_bags = sum(len(entries) for entries in gait_map.values())
            add_section_slide(
                prs, friendly(flipper), "Flipper type",
                f"{flipper_bags} runs across {len(gait_map)} gait families",
            )

            for gait in sorted(gait_map, key=natural_key):
                entries = gait_map[gait]
                add_section_slide(
                    prs, friendly(gait), f"{friendly(flipper)} • Gait",
                    f"{len(entries)} recorded run{'s' if len(entries) != 1 else ''}",
                )
                for run, bag_path in entries:
                    print(f"Reading {bag_path.name}")
                    try:
                        bag_data = read_bag(bag_path)
                        graph_base = plot_dir / f"{flipper}__{gait}__run_{run}"
                        graph_path = save_run_graph(
                            bag_data, graph_base, flipper, gait, run
                        )
                        add_graph_slide(prs, graph_path)
                    except Exception as exc:
                        errors.append((bag_path.name, str(exc)))
                        print(f"ERROR: {bag_path.name}: {exc}")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        prs.save(args.output)
    finally:
        if temporary is not None:
            temporary.cleanup()

    print(f"\nSaved {args.output.resolve()}")
    print(f"Slides: {len(prs.slides)} | Bags graphed: {total_bags - len(errors)} | Errors: {len(errors)}")
    if errors:
        print("Bags skipped:")
        for name, error in errors:
            print(f"  {name}: {error}")


if __name__ == "__main__":
    main()