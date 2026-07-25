#!/usr/bin/env python3
"""Check yaw values around divergence point to confirm AMCL angle bug."""
import csv

rows = list(csv.DictReader(open('/tmp/puppy_nav_data.csv')))
print("frame state              loc_err  rx     ry    ryaw   tx     ty    tyaw(=true_yaw)")
for r in rows[104:125]:
    f = int(r['frame'])
    st = r['state']
    le = float(r['loc_err'])
    rx = float(r['rx'])
    ry = float(r['ry'])
    ryaw = float(r['ryaw'])
    tx = float(r['true_x'])
    ty = float(r['true_y'])
    print(f"{f:5d} {st:20s} {le:7.3f} {rx:6.2f} {ry:6.2f} {ryaw:6.2f} {tx:6.2f} {ty:6.2f}")
