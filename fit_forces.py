#!/usr/bin/env python3
"""
fit_forces.py

Reads a rosbag2 recording (metadata.yaml + .mcap file, both in the same
folder) and fits approximate periodic (Fourier) functions to the Fx, Fy, Fz
channels on the /load_cell_data topic.

Usage:
    python fit_forces.py /path/to/bag_folder
    (bag_folder must contain metadata.yaml and the .mcap file it points to)

Output:
    - Printed fitted equations for Fx(t), Fy(t), Fz(t) with R^2 scores
    - A PNG plot (forces_fit.png) comparing raw data to the fitted curves

Model used:
    F(t) = C + sum_{k=1..N} [ A_k*sin(k*w*t) + B_k*cos(k*w*t) ]

    For each channel:
      1. w (fundamental angular frequency) is estimated independently from
         that channel's own FFT -- channels are NOT forced to share a
         frequency, since e.g. Fz can genuinely oscillate at a different
         rate than Fx/Fy.
      2. w is then refined with a small 1-D grid search in a +/-10% window
         around the FFT estimate. For any *fixed* w, solving for
         A_1..A_N, B_1..B_N, C is a *linear* least-squares problem (the
         basis functions sin(k*w*t), cos(k*w*t) are fixed columns), so
         this search is just "try many w's, solve a linear system each
         time, keep the best". This avoids handing scipy's nonlinear
         curve_fit a hard, non-convex 11-parameter problem where it can
         (and, as seen on Fz, did) converge to the wrong frequency
         entirely while still fitting *some* combination of harmonics.
"""

import sys
import os
import yaml
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def load_bag_metadata(bag_folder):
    meta_path = os.path.join(bag_folder, "metadata.yaml")
    with open(meta_path, "r") as f:
        meta = yaml.safe_load(f)
    info = meta["rosbag2_bagfile_information"]
    mcap_rel_path = info["relative_file_paths"][0]
    mcap_path = os.path.join(bag_folder, mcap_rel_path)
    return mcap_path


def extract_load_cell_data(mcap_path, topic="/load_cell_data"):
    """
    Returns dict of numpy arrays: t, fx, fy, fz, tx, ty, tz

    The LoadCell message batches multiple samples per message:
        header: Header      (one timestamp for the whole batch)
        rows:   int         (number of samples in this batch, e.g. 20)
        cols:   int         (number of channels per sample, = 6)
        data:   float[]     (flattened rows x cols, row-major)

    Column order (confirmed against the plot legend) is:
        [Fx, Fy, Fz, Tx, Ty, Tz]

    Since only one timestamp is given per *message* (not per row), we
    reconstruct per-row timestamps by assuming uniform sampling and
    interpolating between this message's header stamp and the next
    message's header stamp.
    """
    batches = []  # list of (header_stamp_sec, rows, cols, data_array)

    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[topic]
        ):
            stamp = ros_msg.header.stamp
            t_sec = stamp.sec + stamp.nanosec * 1e-9
            arr = np.array(ros_msg.data, dtype=float).reshape(
                ros_msg.rows, ros_msg.cols
            )
            batches.append((t_sec, ros_msg.rows, ros_msg.cols, arr))

    if not batches:
        raise RuntimeError(f"No messages decoded on topic {topic}")

    t0 = batches[0][0]

    t_list, all_rows = [], []
    n_batches = len(batches)
    for i, (t_stamp, rows, cols, arr) in enumerate(batches):
        t_start = t_stamp - t0

        if i < n_batches - 1:
            t_next = batches[i + 1][0] - t0
        else:
            # last batch: reuse previous batch's spacing to extrapolate
            prev_t_start = batches[i - 1][0] - t0
            t_next = t_start + (t_start - prev_t_start)

        # guard against non-monotonic / duplicate timestamps
        span = max(t_next - t_start, 1e-6)
        dt = span / rows

        row_times = t_start + dt * np.arange(rows)
        t_list.append(row_times)
        all_rows.append(arr)

    t = np.concatenate(t_list)
    all_rows = np.concatenate(all_rows, axis=0)  # shape: (N_total_samples, 6)

    return {
        "t": t,
        "fx": all_rows[:, 0],
        "fy": all_rows[:, 1],
        "fz": all_rows[:, 2],
        "tx": all_rows[:, 3],
        "ty": all_rows[:, 4],
        "tz": all_rows[:, 5],
    }


def dominant_frequency(t, signal):
    """Estimate the fundamental angular frequency (rad/s) via FFT,
    with parabolic interpolation across the peak bin for sub-bin
    accuracy. Computed independently per channel -- channels are not
    assumed to share a frequency."""
    dt = np.median(np.diff(t))
    t_uniform = np.arange(t[0], t[-1], dt)
    sig_uniform = np.interp(t_uniform, t, signal)
    sig_uniform = sig_uniform - np.mean(sig_uniform)

    n = len(sig_uniform)
    freqs = np.fft.rfftfreq(n, d=dt)
    fft_mag = np.abs(np.fft.rfft(sig_uniform))
    fft_mag[0] = 0  # ignore DC bin

    peak_idx = np.argmax(fft_mag)

    # parabolic interpolation using the peak bin and its neighbors
    if 0 < peak_idx < len(fft_mag) - 1:
        alpha = fft_mag[peak_idx - 1]
        beta = fft_mag[peak_idx]
        gamma = fft_mag[peak_idx + 1]
        denom = (alpha - 2 * beta + gamma)
        p = 0.5 * (alpha - gamma) / denom if denom != 0 else 0.0
        bin_width = freqs[1] - freqs[0]
        freq_hz = freqs[peak_idx] + p * bin_width
    else:
        freq_hz = freqs[peak_idx]

    return 2 * np.pi * freq_hz  # angular frequency


def _design_matrix(t, w, n_harmonics):
    cols = [np.ones_like(t)]
    for k in range(1, n_harmonics + 1):
        cols.append(np.sin(k * w * t))
        cols.append(np.cos(k * w * t))
    return np.column_stack(cols)


def _linear_fit_at_w(t, signal, w, n_harmonics):
    """Given a FIXED w, solve for C, A_1..A_N, B_1..B_N via ordinary
    linear least squares. No local minima -- this is a plain linear
    system, so it's exact for that w."""
    X = _design_matrix(t, w, n_harmonics)
    coeffs, _, _, _ = np.linalg.lstsq(X, signal, rcond=None)
    fitted = X @ coeffs
    resid = np.sum((signal - fitted) ** 2)
    return coeffs, fitted, resid


def smooth_signal(t, signal, window_frac=0.01):
    """Simple moving-average low-pass filter to reveal the underlying
    periodic trend under the sensor noise. window_frac is the smoothing
    window as a fraction of total samples."""
    n = len(signal)
    window = max(int(n * window_frac), 3)
    if window % 2 == 0:
        window += 1
    kernel = np.ones(window) / window
    return np.convolve(signal, kernel, mode="same")


def fit_channel(t, signal, w_guess, n_harmonics=5, search_frac=0.10, search_points=401):
    """
    Fits F(t) = C + sum_k A_k*sin(k*w*t) + B_k*cos(k*w*t).

    Strategy: refine w with a 1-D grid search in a +/-search_frac window
    around w_guess (the per-channel FFT estimate). For each candidate w,
    the harmonic coefficients are solved exactly via linear least
    squares, and we keep whichever w gave the lowest residual. This
    replaces a single unstable ~11-parameter nonlinear optimization
    (which can converge to the wrong frequency, as happened on Fz) with
    a robust search over the one genuinely nonlinear parameter.
    """
    w_candidates = np.linspace(w_guess * (1 - search_frac),
                                w_guess * (1 + search_frac),
                                search_points)
    best_resid = np.inf
    best_w = w_guess
    for w in w_candidates:
        _, _, resid = _linear_fit_at_w(t, signal, w, n_harmonics)
        if resid < best_resid:
            best_resid = resid
            best_w = w

    coeffs, fitted, _ = _linear_fit_at_w(t, signal, best_w, n_harmonics)
    popt = [best_w] + list(coeffs)  # [w, C, A1, B1, A2, B2, ...]

    # R^2 against raw (noisy) signal
    ss_res = np.sum((signal - fitted) ** 2)
    ss_tot = np.sum((signal - np.mean(signal)) ** 2)
    r2_raw = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    # R^2 against a smoothed version of the signal -- shows how well the
    # fit captures the true underlying periodic trend vs. just noise
    smoothed = smooth_signal(t, signal)
    ss_res_smooth = np.sum((smoothed - fitted) ** 2)
    ss_tot_smooth = np.sum((smoothed - np.mean(smoothed)) ** 2)
    r2_smooth = (
        1 - ss_res_smooth / ss_tot_smooth if ss_tot_smooth > 0 else float("nan")
    )

    return popt, r2_raw, r2_smooth, fitted, smoothed


def format_equation(name, popt, n_harmonics=5):
    w, C = popt[0], popt[1]
    terms = [f"{C:.3f}"]
    for k in range(1, n_harmonics + 1):
        A_k = popt[2 + 2 * (k - 1)]
        B_k = popt[2 + 2 * (k - 1) + 1]
        terms.append(f"{A_k:.3f}*sin({k}*{w:.4f}*t) + {B_k:.3f}*cos({k}*{w:.4f}*t)")
    return f"{name}(t) = " + " + ".join(terms)


def main():
    if len(sys.argv) < 2:
        print("Usage: python fit_forces.py /path/to/bag_folder")
        sys.exit(1)

    bag_folder = sys.argv[1]
    mcap_path = load_bag_metadata(bag_folder)
    print(f"Reading mcap file: {mcap_path}")

    data = extract_load_cell_data(mcap_path)
    t = data["t"]

    n_harmonics = 5
    results = {}
    for name, key in [("Fx", "fx"), ("Fy", "fy"), ("Fz", "fz")]:
        # Each channel gets its OWN frequency estimate -- do not assume
        # Fx/Fy/Fz share a fundamental frequency.
        w_guess = dominant_frequency(t, data[key])
        print(f"{name}: initial FFT frequency guess = {w_guess:.4f} rad/s "
              f"(period ~ {2*np.pi/w_guess:.3f} s)")

        popt, r2_raw, r2_smooth, fitted, smoothed = fit_channel(
            t, data[key], w_guess, n_harmonics=n_harmonics
        )
        results[name] = (popt, r2_raw, r2_smooth, fitted, smoothed)
        eq = format_equation(name, popt, n_harmonics=n_harmonics)
        fitted_w = popt[0]
        print(f"{eq}")
        print(f"    Refined w = {fitted_w:.4f} rad/s (period ~ {2*np.pi/fitted_w:.3f} s)")
        print(f"    R^2 vs raw noisy data:      {r2_raw:.3f}")
        print(f"    R^2 vs smoothed trend:      {r2_smooth:.3f}\n")

    # Plot data vs fit vs smoothed trend
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for ax, (name, key) in zip(axes, [("Fx", "fx"), ("Fy", "fy"), ("Fz", "fz")]):
        popt, r2_raw, r2_smooth, fitted, smoothed = results[name]
        ax.plot(t, data[key], label=f"{name} raw data", alpha=0.3, linewidth=0.6)
        ax.plot(t, smoothed, label=f"{name} smoothed", alpha=0.6, linewidth=1.0)
        ax.plot(t, fitted, label=f"{name} fit (R^2 smooth={r2_smooth:.3f})",
                linewidth=1.5, color="black")
        ax.set_ylabel("Force")
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Load Cell Forces: Raw Data vs. Smoothed Trend vs. Fitted Model")
    fig.tight_layout()
    out_path = os.path.join(os.getcwd(), "forces_fit.png")
    fig.savefig(out_path, dpi=150)
    print(f"Plot saved to: {out_path}")


if __name__ == "__main__":
    main()