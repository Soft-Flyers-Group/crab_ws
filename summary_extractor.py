#!/usr/bin/env python3

"""Extract flipper/gait force maxima from CRAB ROS 2 MCAP bags.

Produces a compact JSON file suitable for building overview slides. Each
20-by-6 LoadCell message is flattened as 20 consecutive samples. For Fx, Fy,
and Fz, "maximum" means the largest sustained value after time-based Hampel
outlier rejection and a rolling-median persistence requirement.
"""

import argparse
import json
from pathlib import Path
import re

import numpy as np
from scipy.ndimage import median_filter
import rosbag2_py
from rclpy.serialization import deserialize_message
from crab_interfaces.msg import LoadCell


FLIPPERS = ("control", "fiberglass", "petg_thin", "rib_flipper", "stripe_flipper")
LOAD_TOPIC = "/load_cell_data"
AXES = ("Fx", "Fy", "Fz")


def identify_flipper(filename):
    lower = filename.lower()
    for flipper in FLIPPERS:
        if lower.startswith(flipper.lower()):
            return flipper
    return None


def parse_gait_and_run(path, flipper):
    """Derive a comparable gait name and repetition number from a filename."""
    stem = path.stem

    if flipper == "fiberglass":
        experiment = stem[len("fiberglass"):].lstrip("_")
        match = re.match(r"^(.*?)(\d+)$", experiment)
        if match:
            gait, run = match.group(1), int(match.group(2))
        else:
            gait, run = experiment, 1
        return gait or "unknown", run

    prefix = f"{flipper}_"
    suffix = f"_{flipper}"
    experiment = stem[len(prefix):] if stem.lower().startswith(prefix.lower()) else stem
    if experiment.lower().endswith(suffix.lower()):
        experiment = experiment[:-len(suffix)]
    if experiment.lower().startswith("vansh_"):
        experiment = experiment[len("vansh_"):]

    match = re.match(r"^(.*)_(\d+)$", experiment)
    if match:
        return match.group(1), int(match.group(2))
    return experiment or "unknown", 1


def odd_window(seconds, sample_rate, minimum=3):
    samples = max(minimum, int(round(seconds * sample_rate)))
    return samples if samples % 2 else samples + 1


def flatten_batches(batch_times, batches):
    """Flatten each rows-by-3 message and reconstruct per-row timestamps."""
    times = np.asarray(batch_times, dtype=np.float64)
    periods = np.diff(times)
    positive = periods[periods > 0]
    fallback = float(np.median(positive)) if positive.size else 0.0
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


def screened_sustained_peak(times, values, hampel_seconds, sustain_seconds,
                            trim_seconds, sigma):
    """Reject local outliers, then select a force sustained in time.

    The Hampel stage compares each sample to a local median using median
    absolute deviation (MAD). Rejected samples are replaced by the local
    median. A longer rolling median then requires the candidate force to
    persist for at least half of the sustain window.
    """
    times = np.asarray(times, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)
    valid = np.isfinite(times) & np.isfinite(values)
    times, values = times[valid], values[valid]
    if values.size < 3:
        return None, {"rejected_samples": 0, "sample_rate_hz": None}

    positive_dt = np.diff(times)
    positive_dt = positive_dt[positive_dt > 0]
    if not positive_dt.size:
        raise ValueError("load-cell timestamps do not advance")
    sample_rate = 1.0 / float(np.median(positive_dt))

    hampel_n = odd_window(hampel_seconds, sample_rate)
    sustain_n = odd_window(sustain_seconds, sample_rate)
    local_median = median_filter(values, size=hampel_n, mode="nearest")
    deviation = np.abs(values - local_median)
    local_mad = median_filter(deviation, size=hampel_n, mode="nearest")
    robust_sigma = 1.4826 * local_mad

    # A small global floor avoids declaring every nonzero deviation an outlier
    # in locally constant regions where MAD becomes exactly zero.
    global_mad = np.median(np.abs(values - np.median(values)))
    sigma_floor = max(1e-9, 0.01 * 1.4826 * global_mad)
    threshold = sigma * np.maximum(robust_sigma, sigma_floor)
    outliers = deviation > threshold
    cleaned = values.copy()
    cleaned[outliers] = local_median[outliers]

    sustained = median_filter(cleaned, size=sustain_n, mode="nearest")
    keep = np.ones(times.size, dtype=bool)
    if trim_seconds > 0 and times[-1] - times[0] > 2 * trim_seconds:
        keep &= times >= times[0] + trim_seconds
        keep &= times <= times[-1] - trim_seconds
    candidate_indices = np.flatnonzero(keep)
    if not candidate_indices.size:
        candidate_indices = np.arange(times.size)
    best_index = candidate_indices[np.argmax(np.abs(sustained[candidate_indices]))]

    return float(sustained[best_index]), {
        "peak_time_s": float(times[best_index] - times[0]),
        "sample_rate_hz": float(sample_rate),
        "hampel_window_samples": int(hampel_n),
        "sustain_window_samples": int(sustain_n),
        "rejected_samples": int(np.count_nonzero(outliers)),
        "rejected_fraction": float(np.mean(outliers)),
        "raw_value_at_peak_time": float(values[best_index]),
        "screened_value": float(sustained[best_index]),
    }


def read_force_peaks(path, screening):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    batch_times = []
    chunks = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != LOAD_TOPIC:
            continue

        msg = deserialize_message(raw, LoadCell)
        values = np.asarray(msg.data, dtype=np.float32)
        expected = msg.rows * msg.cols
        if values.size != expected:
            raise ValueError(
                f"{path.name}: received {values.size} LoadCell values, expected "
                f"{msg.rows}*{msg.cols}={expected}"
            )
        matrix = values.reshape(msg.rows, msg.cols)
        if matrix.shape[1] < 3:
            raise ValueError(f"{path.name}: LoadCell data has fewer than three force axes")
        batch_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        chunks.append(matrix[:, :3])

    if not chunks:
        raise ValueError(f"{path.name}: no {LOAD_TOPIC} messages found")

    times, forces = flatten_batches(batch_times, chunks)
    peaks, diagnostics = {}, {}
    for index, axis in enumerate(AXES):
        peaks[axis], diagnostics[axis] = screened_sustained_peak(
            times, forces[:, index],
            screening["hampel_seconds"], screening["sustain_seconds"],
            screening["trim_seconds"], screening["sigma"],
        )
    return peaks, diagnostics, int(forces.shape[0])


def choose_larger(current, candidate):
    if current is None:
        return candidate
    if candidate["value"] is None:
        return current
    if current["value"] is None or abs(candidate["value"]) > abs(current["value"]):
        return candidate
    return current


def build_summary(bag_dir, screening):
    runs = []
    errors = []

    for path in sorted(bag_dir.glob("*.bag")):
        flipper = identify_flipper(path.name)
        if flipper is None:
            continue
        gait, run = parse_gait_and_run(path, flipper)
        try:
            peaks, diagnostics, sample_count = read_force_peaks(path, screening)
            runs.append({
                "flipper": flipper,
                "gait": gait,
                "run": run,
                "bag": path.name,
                "force_peaks_N": peaks,
                "force_peak_diagnostics": diagnostics,
                "load_sample_count": sample_count,
            })
            print(f"OK    {path.name}")
        except Exception as exc:
            errors.append({"bag": path.name, "error": str(exc)})
            print(f"ERROR {path.name}: {exc}")

    gait_names = sorted({item["gait"] for item in runs}, key=str.lower)
    completion = {
        flipper: {gait: any(
            item["flipper"] == flipper and item["gait"] == gait for item in runs
        ) for gait in gait_names}
        for flipper in FLIPPERS
    }

    flipper_summaries = {}
    for flipper in FLIPPERS:
        flipper_runs = [item for item in runs if item["flipper"] == flipper]
        by_gait = {}
        overall = {axis: None for axis in AXES}

        for gait in sorted({item["gait"] for item in flipper_runs}, key=str.lower):
            gait_runs = [item for item in flipper_runs if item["gait"] == gait]
            gait_axes = {}
            for axis in AXES:
                best = None
                for item in gait_runs:
                    candidate = {
                        "value": item["force_peaks_N"][axis],
                        "run": item["run"],
                        "bag": item["bag"],
                    }
                    best = choose_larger(best, candidate)
                    overall_candidate = {**candidate, "gait": gait}
                    overall[axis] = choose_larger(overall[axis], overall_candidate)
                gait_axes[axis] = best
            by_gait[gait] = {
                "run_count": len(gait_runs),
                "max_abs_force_N": gait_axes,
            }

        flipper_summaries[flipper] = {
            "bag_count": len(flipper_runs),
            "overall_max_abs_force_N": overall,
            "gaits": by_gait,
        }

    return {
        "metric": "largest absolute screened sustained force, signed value retained",
        "peak_filter": {
            "type": "time-based Hampel outlier rejection plus rolling median",
            **screening,
            "purpose": "reject noisy spikes and require a sustained force",
        },
        "units": "N",
        "bag_directory": str(bag_dir),
        "flippers": list(FLIPPERS),
        "gaits": gait_names,
        "completion_matrix": completion,
        "flipper_summaries": flipper_summaries,
        "runs": runs,
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag-dir", type=Path,
        default=Path("/home/odinroast/crab_ws/bags"),
        help="Directory containing .bag MCAP files",
    )
    parser.add_argument(
        "--output", type=Path,
        default=Path("flipper_force_summary.json"),
        help="Output JSON path",
    )
    parser.add_argument("--hampel-seconds", type=float, default=0.10,
                        help="Local Hampel outlier window in seconds (default: 0.10)")
    parser.add_argument("--sustain-seconds", type=float, default=0.25,
                        help="Required rolling-median persistence in seconds (default: 0.25)")
    parser.add_argument("--trim-seconds", type=float, default=0.50,
                        help="Time ignored at both recording ends (default: 0.50)")
    parser.add_argument("--hampel-sigma", type=float, default=3.5,
                        help="Hampel rejection threshold in robust sigma (default: 3.5)")
    args = parser.parse_args()

    if not args.bag_dir.is_dir():
        raise SystemExit(f"Bag directory does not exist: {args.bag_dir}")

    if args.hampel_seconds <= 0 or args.sustain_seconds <= 0:
        raise SystemExit("Hampel and sustain windows must be greater than zero")
    if args.trim_seconds < 0 or args.hampel_sigma <= 0:
        raise SystemExit("Trim must be nonnegative and Hampel sigma must be positive")

    screening = {
        "hampel_seconds": args.hampel_seconds,
        "sustain_seconds": args.sustain_seconds,
        "trim_seconds": args.trim_seconds,
        "sigma": args.hampel_sigma,
    }
    summary = build_summary(args.bag_dir, screening)
    args.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\nWrote {args.output.resolve()}")
    print(f"Processed {len(summary['runs'])} bags; {len(summary['errors'])} errors")


if __name__ == "__main__":
    main()