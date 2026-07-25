#!/usr/bin/env python3
"""Detail analysis of frame 100-130 to find AMCL divergence root cause."""
import csv

rows = list(csv.DictReader(open('/tmp/puppy_nav_data.csv')))
print("frame state              loc_err   rx     ry     tx     ty     v     w")
for r in rows[100:135]:
    f = int(r['frame'])
    st = r['state']
    le = float(r['loc_err'])
    rx = float(r['rx'])
    ry = float(r['ry'])
    tx = float(r['true_x'])
    ty = float(r['true_y'])
    v = float(r['dwa_v'])
    w = float(r['dwa_w'])
    print(f"{f:5d} {st:20s} {le:7.3f} {rx:6.2f} {ry:6.2f} {tx:6.2f} {ty:6.2f} {v:5.2f} {w:6.2f}")
