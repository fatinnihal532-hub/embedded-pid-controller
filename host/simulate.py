#!/usr/bin/env python3
"""Offline copy of the firmware controller and plant.

The equations here are the same ones in firmware/sketch.ino, line for line.
Tuning on the laptop first and only then flashing the board is the normal
workflow in control work: a run that takes four minutes on hardware takes
milliseconds here, so a gain sweep is cheap.

    python host/simulate.py --kp 8 --ki 0.45 --kd 12 --out data/tuned.csv
"""
import argparse
import pathlib

DT = 0.1            # control period, seconds
T_AMBIENT = 25.0
PLANT_K = 60.0      # degrees C rise at full power
PLANT_TAU = 25.0    # seconds
DEAD_SLOTS = 21     # 2.0 s of transport delay
OUT_MIN, OUT_MAX = 0.0, 100.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def run(kp, ki, kd, setpoint, duration, disturbance_at=None, disturbance=0.0,
        anti_windup=True):
    """Return a list of rows matching the firmware's CSV columns."""
    integral = 0.0
    last_meas = T_AMBIENT
    first = True
    temperature = T_AMBIENT
    delay_line = [0.0] * DEAD_SLOTS
    idx = 0
    dist = 0.0
    rows = []

    for k in range(int(duration / DT)):
        t = k * DT
        if disturbance_at is not None and t >= disturbance_at:
            dist = disturbance

        measured = temperature

        # ---- controller ----
        error = setpoint - measured
        p = kp * error
        if first:
            last_meas = measured
            first = False
        d = -kd * (measured - last_meas) / DT
        last_meas = measured

        unsaturated = p + integral + d
        if anti_windup:
            winding_up = ((unsaturated >= OUT_MAX and error > 0) or
                          (unsaturated <= OUT_MIN and error < 0))
            if not winding_up:
                integral = clamp(integral + ki * error * DT, OUT_MIN - 20.0, OUT_MAX)
        else:
            # the naive version: integrate no matter what the output is doing
            integral += ki * error * DT

        output = clamp(p + integral + d, OUT_MIN, OUT_MAX)

        # ---- plant ----
        delay_line[idx] = output
        idx = (idx + 1) % DEAD_SLOTS
        delayed = delay_line[idx]
        drive = PLANT_K * delayed / 100.0
        temperature += (-(temperature - T_AMBIENT - dist) + drive) / PLANT_TAU * DT

        rows.append((t, setpoint, measured, output, p, integral, d))

    return rows


def write_csv(rows, path):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        f.write("seconds,setpoint,temperature,output_pct,p_term,i_term,d_term\n")
        for r in rows:
            f.write("{:.1f},{:.2f},{:.3f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(*r))
    print(f"wrote {len(rows)} rows to {path}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--kp", type=float, default=8.0)
    ap.add_argument("--ki", type=float, default=0.45)
    ap.add_argument("--kd", type=float, default=12.0)
    ap.add_argument("--sp", type=float, default=45.0)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--disturbance-at", type=float, default=None)
    ap.add_argument("--disturbance", type=float, default=0.0)
    ap.add_argument("--no-anti-windup", action="store_true",
                    help="integrate even while the output is saturated")
    ap.add_argument("--out", default="data/run.csv")
    args = ap.parse_args()

    rows = run(args.kp, args.ki, args.kd, args.sp, args.duration,
               args.disturbance_at, args.disturbance,
               anti_windup=not args.no_anti_windup)
    write_csv(rows, args.out)


if __name__ == "__main__":
    main()
