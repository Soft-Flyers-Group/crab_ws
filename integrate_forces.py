#!/usr/bin/env python3
"""
integrate_forces.py

Computes the integral (impulse) of the Fx, Fy, Fz load-cell forces over
one period of the periodic motion, using the fitted Fourier model from
fit_forces.py.

Usage:
    python integrate_forces.py /path/to/bag_folder

Requires fit_forces.py to be in the SAME FOLDER as this script (it
imports the bag-reading and fitting functions from it, so both scripts
stay consistent with each other).

What "integral over one period" means here:
    integral_0^T F(t) dt   where T = 2*pi/w (one full cycle of that
    channel's fitted fundamental frequency).

    Because sin(k*w*t) and cos(k*w*t) each complete a whole number of
    cycles over exactly one period T, every harmonic term integrates to
    exactly zero over that span. So the integral of a periodic signal
    over one full period always collapses to just C * T, where C is the
    fitted constant/mean term. This is true for ANY periodic signal, not
    an artifact of the fit -- the net impulse over an integer number of
    periods equals the average force times the elapsed time.

    If C is small/near-zero (a channel that mostly oscillates around
    zero), this number will be small even though the force clearly swings
    through large positive and negative values each cycle -- that's
    expected, since the positive and negative halves of the cycle cancel.
    That's why this script also reports the net impulse over the WHOLE
    recording and plots the running (cumulative) integral over time --
    that plot will reveal any actual drift/asymmetry that a single-period
    number can't show.

    Raw integral (1 period): rather than trusting a single window (e.g.
    just the first period, which can be skewed by startup transients as
    the mechanism spins up / the sensor settles), this script skips the
    first couple periods and then walks forward through the rest of the
    recording in consecutive, NON-OVERLAPPING one-period windows,
    reporting their mean +/- std (and how many periods n were used).
    The std tells you how much the single-period raw integral actually
    varies cycle-to-cycle -- a small std means the number is stable and
    trustworthy, a large std means you should look at the cumulative
    integral plot more closely.

Output (printed table), per channel:
    - T              : fitted period (s)
    - Model integral : C * T, the analytic integral of the fitted model
                       over exactly one period
    - Raw integral   : mean trapezoidal-rule integral of the ACTUAL raw
                       data over one period, averaged over consecutive
                       non-overlapping periods sampled after skipping
                       the first couple periods of the recording (not
                       just the first period), as a check against the
                       model number above. Std across those periods
                       (and the count n used) is also reported.
    - Total impulse  : trapezoidal-rule integral of the raw data over the
                       ENTIRE recording

Also saves forces_cumulative_integral.png: the running integral of each
channel's raw data over the whole recording, so you can see whether the
net force trends to zero, drifts, or oscillates around some nonzero mean.
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fit_forces import (
    load_bag_metadata,
    extract_load_cell_data,
    dominant_frequency,
    fit_channel,
)

# np.trapz was removed in numpy 2.0+ in favor of np.trapezoid. Use
# whichever is available so this runs on either numpy version.
_trapz = getattr(np, "trapezoid", None) or np.trapz

# Number of periods to skip at the start of the recording before
# sampling windows. Early cycles are often still ramping up / settling
# (startup transient) and can be way off from steady-state, inflating
# the std -- so we ignore this many periods' worth of time first.
SKIP_PERIODS = 2


def integrate_one_period(t, signal, w, t_start=None):
    """Numerically integrate `signal` (trapezoidal rule) over one period
    T = 2*pi/w, starting at t_start (defaults to t[0])."""
    if t_start is None:
        t_start = t[0]
    T = 2 * np.pi / w
    t_end = t_start + T
    mask = (t >= t_start) & (t <= t_end)
    if mask.sum() < 2:
        raise ValueError("Not enough samples in one period to integrate.")
    return _trapz(signal[mask], t[mask]), T


def sample_period_integrals(t, signal, w):
    """Integrate `signal` over one period T, repeated over consecutive,
    NON-OVERLAPPING periods walking forward through the recording (so
    each sample is a genuinely separate cycle, not an overlapping
    snapshot of a shorter stretch).

    The first SKIP_PERIODS periods (likely startup transient) are
    skipped entirely. From there, windows are placed back-to-back --
    [t_lo, t_lo+T], [t_lo+T, t_lo+2T], [t_lo+2T, t_lo+3T], ... -- for as
    many whole periods as fit in the remaining recording. If skipping
    leaves no room for even one full period, falls back to not
    skipping, and if the recording is *still* too short for one period,
    falls back to a single window starting at t[0].

    Returns (mean_integral, std_integral, T, n_periods_used,
    list_of_(t_start, integral)).
    """
    T = 2 * np.pi / w
    t_hi = t[-1]
    t_lo = t[0] + SKIP_PERIODS * T

    if t_lo + T > t_hi:
        # Not enough recording left after skipping -- fall back to not
        # skipping the startup periods.
        t_lo = t[0]

    if t_lo + T > t_hi:
        # Recording barely covers one period even without skipping --
        # just use what we have.
        integral, T = integrate_one_period(t, signal, w, t_start=t_lo)
        return integral, 0.0, T, 1, [(t_lo, integral)]

    n_periods = int(np.floor((t_hi - t_lo) / T))
    starts = t_lo + np.arange(n_periods) * T
    results = []
    for t_start in starts:
        integral, _ = integrate_one_period(t, signal, w, t_start=t_start)
        results.append((t_start, integral))

    values = np.array([v for _, v in results])
    return values.mean(), values.std(), T, n_periods, results


def cumulative_integral(t, signal):
    """Running (cumulative) integral of `signal` over time, trapezoidal."""
    running = np.zeros_like(signal)
    running[1:] = np.cumsum((signal[1:] + signal[:-1]) / 2.0 * np.diff(t))
    return running


def main():
    if len(sys.argv) < 2:
        print("Usage: python integrate_forces.py /path/to/bag_folder")
        sys.exit(1)

    bag_folder = sys.argv[1]
    mcap_path = load_bag_metadata(bag_folder)
    print(f"Reading mcap file: {mcap_path}")

    data = extract_load_cell_data(mcap_path)
    t = data["t"]

    n_harmonics = 5
    channels = [("Fx", "fx"), ("Fy", "fy"), ("Fz", "fz")]

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    header = (f"{'Channel':<8}{'T (s)':<10}{'Model integral (C*T)':<24}"
              f"{'Raw integral (1 period, mean+/-std, n)':<40}{'Total impulse (full run)':<26}")
    print("\n" + header)
    print("-" * len(header))

    for ax, (name, key) in zip(axes, channels):
        signal = data[key]

        # Fit fresh so this script is self-contained / doesn't depend on
        # fit_forces.py having already been run.
        w_guess = dominant_frequency(t, signal)
        popt, r2_raw, r2_smooth, fitted, smoothed = fit_channel(
            t, signal, w_guess, n_harmonics=n_harmonics
        )
        w_fit, C = popt[0], popt[1]
        T = 2 * np.pi / w_fit

        # Analytic integral of the fitted model over one period: all
        # harmonic (sin/cos) terms vanish over a full period, leaving
        # just the constant term times the period length.
        model_integral = C * T

        # Cross-check: numerically integrate the RAW data over one
        # period, sampled over consecutive NON-OVERLAPPING periods
        # walking through the recording (skipping the first
        # SKIP_PERIODS as likely startup transient) to avoid skew and
        # to see how much this number varies cycle-to-cycle.
        raw_mean, raw_std, _, n_periods, _ = sample_period_integrals(t, signal, w_fit)

        # Net impulse over the whole recording (raw data).
        total_impulse = _trapz(signal, t)

        raw_str = f"{raw_mean:.4f} +/- {raw_std:.4f} (n={n_periods})"
        print(f"{name:<8}{T:<10.4f}{model_integral:<24.4f}"
              f"{raw_str:<38}{total_impulse:<26.4f}")

        running = cumulative_integral(t, signal)
        ax.plot(t, running, color="black", linewidth=1.2)
        ax.axhline(0, color="gray", linewidth=0.6, linestyle="--")
        ax.set_ylabel(f"{name} cumulative\nintegral")

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Cumulative (Running) Integral of Load Cell Forces Over Time")
    fig.tight_layout()
    out_path = os.path.join(os.getcwd(), "forces_cumulative_integral.png")
    fig.savefig(out_path, dpi=150)
    print(f"\nCumulative integral plot saved to: {out_path}")


if __name__ == "__main__":
    main()