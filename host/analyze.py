#!/usr/bin/env python3
"""Measure the step response in a logged run and plot it.

Works on a CSV produced either by the firmware over the serial port or by
host/simulate.py, because both write the same columns.

    python host/analyze.py data/tuned.csv
    python host/analyze.py data/sluggish.csv data/tuned.csv data/aggressive.csv \
        --labels "low gain" "tuned" "high gain" --out docs/step_response.png

Reported for each run:
    rise time        10 % to 90 % of the final step
    overshoot        peak above the set point, as a percentage of the step
    settling time    last time the signal enters and stays inside +/- 2 %
    steady error     set point minus the average of the last 10 seconds
"""
import argparse
import csv
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SERIES_COLORS = ["#1f6feb", "#c2410c", "#0f766e"]
GRID = "#e5e3df"
INK = "#3d3a35"
MUTED = "#8a857d"


def load(path):
    t, sp, pv, out = [], [], [], []
    with pathlib.Path(path).open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                t.append(float(row["seconds"]))
                sp.append(float(row["setpoint"]))
                pv.append(float(row["temperature"]))
                out.append(float(row["output_pct"]))
            except (KeyError, ValueError):
                continue
    if not t:
        raise SystemExit(f"no usable rows in {path}")
    return t, sp, pv, out


def metrics(t, sp, pv):
    start = pv[0]
    target = sp[-1]
    span = target - start
    if abs(span) < 1e-6:
        return None

    def crossing(fraction):
        level = start + fraction * span
        for time, value in zip(t, pv):
            if (span > 0 and value >= level) or (span < 0 and value <= level):
                return time
        return None

    t10, t90 = crossing(0.10), crossing(0.90)
    rise = None if t10 is None or t90 is None else t90 - t10

    peak = max(pv) if span > 0 else min(pv)
    overshoot = (peak - target) / span * 100.0
    overshoot = max(overshoot, 0.0)

    band = 0.02 * abs(span)
    settle = None
    for i in range(len(t) - 1, -1, -1):
        if abs(pv[i] - target) > band:
            settle = t[i + 1] if i + 1 < len(t) else None
            break
    else:
        settle = t[0]

    tail = [v for time, v in zip(t, pv) if time >= t[-1] - 10.0]
    steady_error = target - (sum(tail) / len(tail))

    return {
        "rise_s": rise,
        "overshoot_pct": overshoot,
        "settle_s": settle,
        "steady_error_c": steady_error,
        "peak_c": peak,
        "target_c": target,
    }


def fmt(value, unit="", nd=2):
    return "n/a" if value is None else f"{value:.{nd}f}{unit}"


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9, length=0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--labels", nargs="*", default=None)
    ap.add_argument("--out", default="docs/step_response.png")
    args = ap.parse_args()

    labels = args.labels or [pathlib.Path(p).stem for p in args.csv]
    if len(labels) != len(args.csv):
        raise SystemExit("give one label per CSV file")

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9, 6), sharex=True,
        gridspec_kw={"height_ratios": [1.6, 1], "hspace": 0.2},
    )
    fig.patch.set_facecolor("#fcfcfb")

    print(f"{'run':<14}{'rise':>10}{'overshoot':>12}{'settling':>11}{'steady err':>13}")
    print("-" * 60)

    target = None
    for n, (path, label) in enumerate(zip(args.csv, labels)):
        color = SERIES_COLORS[n % len(SERIES_COLORS)]
        t, sp, pv, out = load(path)
        target = sp[-1]

        ax1.plot(t, pv, color=color, linewidth=2, label=label)
        ax2.plot(t, out, color=color, linewidth=2, label=label)

        m = metrics(t, sp, pv)
        if m:
            print(f"{label:<14}{fmt(m['rise_s'], ' s'):>10}"
                  f"{fmt(m['overshoot_pct'], ' %'):>12}"
                  f"{fmt(m['settle_s'], ' s'):>11}"
                  f"{fmt(m['steady_error_c'], ' C'):>13}")
            if len(args.csv) == 1:
                ax1.annotate(label, (t[-1], pv[-1]), xytext=(6, 0),
                             textcoords="offset points", color=color,
                             fontsize=9.5, fontweight="bold", va="center")

    if target is not None:
        ax1.axhline(target, color=MUTED, linewidth=1.4, linestyle="--")
        ax1.annotate(f"set point {target:.0f} °C", (0, target),
                     xytext=(4, 6), textcoords="offset points",
                     color=MUTED, fontsize=9)

    ax1.set_ylabel("Temperature (°C)", color=INK, fontsize=10)
    ax1.set_title("PID step response", color=INK, fontsize=13,
                  fontweight="bold", loc="left", pad=12)
    if len(args.csv) > 1:
        ax1.legend(frameon=False, loc="lower right", fontsize=9, labelcolor=INK)
    style(ax1)

    ax2.set_ylabel("Heater output (%)", color=INK, fontsize=10)
    ax2.set_xlabel("Time (seconds)", color=INK, fontsize=10)
    ax2.set_ylim(-5, 105)
    style(ax2)

    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    right = 0.86 if len(args.csv) == 1 else 0.97
    fig.subplots_adjust(left=0.085, right=right, top=0.9, bottom=0.1)
    fig.savefig(out_path, dpi=160)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
