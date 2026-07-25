#!/usr/bin/env python3
"""Quick analysis of localization error trends."""
import csv

for method in ['proposed_amcl_dwa', 'proposed_amcl_teb']:
    print(f'=== {method} ===')
    with open(f'/tmp/eval/{method}_data.csv') as f:
        r = csv.DictReader(f)
        rows = list(r)
    # Sample every 100 frames
    for i in range(0, len(rows), 100):
        row = rows[i]
        print(f'  f={row["frame"]:>4} t={row["time"]:>6} '
              f'x={float(row["x"]):.2f} y={float(row["y"]):.2f} '
              f'state={row["state"]} cov={float(row["coverage"]):.1f}% '
              f'loc_err={float(row["loc_error"]):.3f} '
              f'conf={float(row["loc_conf"]):.3f}')
    # Last frame
    row = rows[-1]
    print(f'  f={row["frame"]:>4} t={row["time"]:>6} '
          f'x={float(row["x"]):.2f} y={float(row["y"]):.2f} '
          f'state={row["state"]} cov={float(row["coverage"]):.1f}% '
          f'loc_err={float(row["loc_error"]):.3f} '
          f'conf={float(row["loc_conf"]):.3f}')
    # Stats
    loc_errs = [float(r['loc_error']) for r in rows]
    mean_err = sum(loc_errs) / len(loc_errs)
    print(f'  LocErr: mean={mean_err:.3f} max={max(loc_errs):.3f} '
          f'min={min(loc_errs):.3f}')
    high = sum(1 for e in loc_errs if e > 0.5)
    print(f'  Frames with LocErr > 0.5m: {high}/{len(loc_errs)} '
          f'({100*high/len(loc_errs):.1f}%)')
    # Find when error spikes
    prev_err = 0
    for i, row in enumerate(rows):
        err = float(row['loc_error'])
        if err > 0.5 and prev_err < 0.3:
            print(f'  ERROR SPIKE at frame {row["frame"]}: '
                  f'{prev_err:.3f} -> {err:.3f} '
                  f'pos=({float(row["x"]):.2f},{float(row["y"]):.2f})')
        prev_err = err
    print()
