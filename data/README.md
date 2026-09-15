The three runs behind the table in the main README are written here.

They are generated, not committed, because `host/simulate.py` reproduces them
exactly: the plant model and the controller are both deterministic.

```bash
python host/simulate.py --kp 2 --ki 0.08 --kd 0 --sp 70 --duration 200 --out data/low_gain.csv
python host/simulate.py --kp 6 --ki 0.30 --kd 8 --sp 70 --duration 200 --out data/tuned.csv
python host/simulate.py --kp 6 --ki 0.30 --kd 8 --sp 70 --duration 200 --no-anti-windup \
    --out data/windup.csv
```

Then rebuild the figure:

```bash
python host/analyze.py data/low_gain.csv data/tuned.csv data/windup.csv \
    --labels "low gain" "tuned" "windup unguarded" --out docs/step_response.svg
```
