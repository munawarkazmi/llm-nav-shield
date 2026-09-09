#!/usr/bin/env python3
"""Build costmap pairs that straddle a localisation correction.

The earlier bag tools work in the odom frame, where drift accumulates and
nothing ever corrects it. This one uses a bag whose transform tree carries
a live map frame, so the localiser is continuously correcting odometry,
and asks the question that only a map frame can pose:

  when localisation jumps, does a plan the shield already passed stay safe?

For each correction above a threshold, two costmaps are built from the
same scans over the same window, one at the robot's pose just before the
jump and one just after. Both are in the robot's own frame, so the
accumulated world shifts underneath the robot by exactly the correction.
A plan verified against the first can then be re-verified against the
second.

The bag uses the legacy tf/tfMessage type, which rosbags will not resolve,
so tf and LaserScan are decoded from raw bytes here.

Data (not committed, 31.6 MB):
  http://download.ros.org/data/graph_slam/wg-cafe.bag
  PR2 at the Willow Garage cafe, 322 s, map -> odom_combined -> base_footprint

Usage:
  python3 tools/bag_map_scenarios.py --bag wg-cafe.bag --out /tmp/map_scen \\
      --count 30 --min-jump 0.02
"""
import argparse
import bisect
import json
import math
import pathlib
import struct
import sys

import numpy as np
from rosbags.highlevel import AnyReader

FREE, LETHAL, UNKNOWN = 0, 254, 255


def decode_tf(buf):
    out, i = [], 0
    n, = struct.unpack_from("<I", buf, i)
    i += 4
    for _ in range(n):
        _seq, sec, nsec = struct.unpack_from("<III", buf, i)
        i += 12
        ln, = struct.unpack_from("<I", buf, i)
        i += 4
        frame = buf[i:i + ln].decode("ascii", "replace").lstrip("/")
        i += ln
        ln, = struct.unpack_from("<I", buf, i)
        i += 4
        child = buf[i:i + ln].decode("ascii", "replace").lstrip("/")
        i += ln
        x, y, _z, qx, qy, qz, qw = struct.unpack_from("<7d", buf, i)
        i += 56
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        out.append((frame, child, sec + nsec * 1e-9, x, y, yaw))
    return out


def decode_scan(buf):
    i = 12
    ln, = struct.unpack_from("<I", buf, i)
    i += 4
    frame = buf[i:i + ln].decode("ascii", "replace").lstrip("/")
    i += ln
    amin, amax, ainc, _ti, _st, rmin, rmax = struct.unpack_from("<7f", buf, i)
    i += 28
    n, = struct.unpack_from("<I", buf, i)
    i += 4
    ranges = np.frombuffer(buf, dtype="<f4", count=n, offset=i).astype(np.float64)
    return {"frame": frame, "angle_min": amin, "angle_increment": ainc,
            "range_min": rmin, "range_max": rmax, "ranges": ranges}


def compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at),
            ay + bx * math.sin(at) + by * math.cos(at), at + bt)


def invert(t):
    x, y, th = t
    c, s = math.cos(-th), math.sin(-th)
    return (-(x * c - y * s), -(x * s + y * c), -th)


class Series:
    """One frame pair over time, linearly interpolated."""

    def __init__(self):
        self.t, self.v = [], []

    def add(self, t, x, y, th):
        self.t.append(t)
        self.v.append((x, y, th))

    def sort(self):
        z = sorted(zip(self.t, self.v))
        self.t = [a for a, _ in z]
        self.v = [b for _, b in z]

    def at(self, t):
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i == 0:
            return self.v[0]
        if i >= len(self.t):
            return self.v[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        a, b = self.v[i - 1], self.v[i]
        f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        d = (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi
        return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * d)


def integrate(hits, passes, scan, pose, cells, res, max_reach):
    lx, ly, lth = pose
    n = len(scan["ranges"])
    ang = scan["angle_min"] + np.arange(n) * scan["angle_increment"] + lth
    rng = scan["ranges"]
    good = np.isfinite(rng) & (rng >= scan["range_min"]) & (rng <= scan["range_max"])
    reach = np.minimum(np.where(good, rng, max_reach), max_reach)
    step = res * 0.5
    d = np.arange(int(max_reach / step) + 1) * step
    on = d[None, :] <= reach[:, None]
    cx = np.floor((lx + d[None, :] * np.cos(ang)[:, None]) / res).astype(np.int64)
    cy = np.floor((ly + d[None, :] * np.sin(ang)[:, None]) / res).astype(np.int64)
    ok = on & (cx >= 0) & (cx < cells) & (cy >= 0) & (cy < cells)
    np.add.at(passes, (cy[ok], cx[ok]), 1)
    hx = np.floor((lx + rng[good] * np.cos(ang[good])) / res).astype(np.int64)
    hy = np.floor((ly + rng[good] * np.sin(ang[good])) / res).astype(np.int64)
    m = (hx >= 0) & (hx < cells) & (hy >= 0) & (hy < cells)
    np.add.at(hits, (hy[m], hx[m]), 1)


def to_ascii(grid, block):
    cells = grid.shape[0]
    rows = cells // block
    out = []
    for r in range(rows):
        y0 = cells - (r + 1) * block
        line = []
        for cx in range(rows):
            b = grid[y0:y0 + block, cx * block:(cx + 1) * block]
            line.append("#" if (b == LETHAL).any()
                        else ("?" if (b == UNKNOWN).any() else "."))
        out.append("".join(line))
    return out


def clearance_ok(grid, x, y, r_cells):
    cells = grid.shape[0]
    r = int(math.ceil(r_cells))
    if x - r < 0 or y - r < 0 or x + r >= cells or y + r >= cells:
        return False
    sub = grid[y - r:y + r + 1, x - r:x + r + 1]
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return bool((sub[(xx * xx + yy * yy) <= r_cells * r_cells] == FREE).all())


def pick_goal(grid, sx, sy, r_cells):
    ys, xs = np.nonzero(grid == FREE)
    for i in np.argsort(-((xs - sx) ** 2 + (ys - sy) ** 2)):
        x, y = int(xs[i]), int(ys[i])
        if clearance_ok(grid, x, y, r_cells):
            return (x, y)
    return None


def write_pgm(path, grid, res):
    cells = grid.shape[0]
    with open(path, "wb") as f:
        f.write(f"P5\n# resolution {res:g} origin 0 0\n{cells} {cells}\n255\n".encode())
        f.write(grid.tobytes())


def build(scans, times, chain, key_pose, kt, window, cells, res, radius):
    half = cells / 2.0 * res
    max_reach = half * 1.45
    hits = np.zeros((cells, cells), np.int32)
    passes = np.zeros((cells, cells), np.int32)
    inv = invert(key_pose)
    lo = bisect.bisect_left(times, kt - window)
    hi = bisect.bisect_right(times, kt)
    used = 0
    for s in scans[lo:hi]:
        lp = chain(s["t"], s["frame"])
        if lp is None:
            continue
        rel = compose(inv, lp)
        integrate(hits, passes, s, (rel[0] + half, rel[1] + half, rel[2]),
                  cells, res, max_reach)
        used += 1
    grid = np.full((cells, cells), UNKNOWN, np.uint8)
    grid[passes > 0] = FREE
    grid[hits > 0] = LETHAL
    fp = int(math.ceil((radius + res) / res))
    c = cells // 2
    yy, xx = np.ogrid[-fp:fp + 1, -fp:fp + 1]
    grid[c - fp:c + fp + 1, c - fp:c + fp + 1][(xx * xx + yy * yy) <= fp * fp] = FREE
    return grid, used


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scan-topic", default="base_scan")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom_combined")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--count", type=int, default=30)
    ap.add_argument("--min-jump", type=float, default=0.02)
    ap.add_argument("--cells", type=int, default=200)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--ascii-block", type=int, default=5)
    ap.add_argument("--robot-radius", type=float, default=0.105)
    ap.add_argument("--window-s", type=float, default=3.0)
    args = ap.parse_args()

    out = pathlib.Path(args.out)
    for sub in ("before", "after"):
        (out / sub / "scenarios").mkdir(parents=True, exist_ok=True)

    series, scans = {}, []
    with AnyReader([pathlib.Path(args.bag)]) as r:
        tf_conns = [c for c in r.connections if c.topic.lstrip("/") == "tf"]
        for conn, ts, raw in r.messages(connections=tf_conns):
            for f, c, t, x, y, yaw in decode_tf(raw):
                series.setdefault((f, c), Series()).add(t, x, y, yaw)
        sc = [c for c in r.connections if c.topic.lstrip("/") == args.scan_topic]
        for conn, ts, raw in r.messages(connections=sc):
            m = decode_scan(raw)
            m["t"] = ts / 1e9
            scans.append(m)
    for s in series.values():
        s.sort()
    scans.sort(key=lambda s: s["t"])
    times = [s["t"] for s in scans]
    print(f"tf pairs {len(series)}, scans {len(scans)}")

    mo = series.get((args.map_frame, args.odom_frame))
    ob = series.get((args.odom_frame, args.base_frame))
    if mo is None or ob is None:
        print("missing map or odom transform", file=sys.stderr)
        return 2

    laser_frame = scans[0]["frame"]
    bl = series.get((args.base_frame, laser_frame))
    if bl is None:
        print(f"no {args.base_frame} -> {laser_frame}", file=sys.stderr)
        return 2

    def map_base(t):
        a, b = mo.at(t), ob.at(t)
        return None if a is None or b is None else compose(a, b)

    def chain(t, _frame):
        mb = map_base(t)
        l = bl.at(t)
        return None if mb is None or l is None else compose(mb, l)

    # Correction events: consecutive map->odom samples that move.
    jumps = []
    for i in range(1, len(mo.t)):
        d = math.hypot(mo.v[i][0] - mo.v[i - 1][0], mo.v[i][1] - mo.v[i - 1][1])
        if d >= args.min_jump:
            jumps.append((d, mo.t[i - 1], mo.t[i]))
    jumps.sort(reverse=True)
    print(f"corrections >= {args.min_jump * 1000:.0f} mm: {len(jumps)}, "
          f"largest {jumps[0][0] * 1000:.0f} mm" if jumps else "no corrections")

    res, cells = args.resolution, args.cells
    infl = float(math.ceil((args.robot_radius + res) / res))
    made, index = 0, []
    for d, t_before, t_after in jumps:
        if made >= args.count:
            break
        if t_before - args.window_s < times[0] or t_after > times[-1]:
            continue
        pb, pa = map_base(t_before), map_base(t_after)
        if pb is None or pa is None:
            continue
        gb, nb = build(scans, times, chain, pb, t_before, args.window_s,
                       cells, res, args.robot_radius)
        ga, na = build(scans, times, chain, pa, t_after, args.window_s,
                       cells, res, args.robot_radius)
        if nb < 3 or na < 3:
            continue
        c = cells // 2
        if not clearance_ok(gb, c, c, infl) or not clearance_ok(ga, c, c, infl):
            continue
        goal = pick_goal(gb, c, c, infl)
        if goal is None:
            continue
        start_m = ((c + 0.5) * res, (c + 0.5) * res)
        goal_m = ((goal[0] + 0.5) * res, (goal[1] + 0.5) * res)
        changed = float((gb != ga).mean())
        for tag, grid, used in (("before", gb, nb), ("after", ga, na)):
            scen = {
                "id": made, "width": cells, "height": cells, "resolution": res,
                "origin": [0.0, 0.0],
                "ascii_cell_m": res * args.ascii_block,
                "start": [round(start_m[0], 3), round(start_m[1], 3)],
                "goal": [round(goal_m[0], 3), round(goal_m[1], 3)],
                "correction_m": round(d, 4), "phase": tag,
                "scans_accumulated": used,
                "unknown_fraction": round(float((grid == UNKNOWN).mean()), 4),
                "ascii": to_ascii(grid, args.ascii_block),
            }
            base = out / tag / "scenarios" / f"scenario_{made:02d}"
            base.with_suffix(".json").write_text(json.dumps(scen, indent=2))
            write_pgm(base.with_suffix(".pgm"), grid, res)
        index.append({"id": made, "correction_m": d, "cells_changed": changed,
                      "unknown_before": float((gb == UNKNOWN).mean())})
        made += 1

    (out / "index.json").write_text(json.dumps(index, indent=2))
    if index:
        cor = sorted(r["correction_m"] for r in index)
        ch = sorted(r["cells_changed"] for r in index)
        print(f"wrote {made} before/after pairs to {out}")
        print(f"  correction   min {cor[0]*1000:.0f} mm  median "
              f"{cor[len(cor)//2]*1000:.0f} mm  max {cor[-1]*1000:.0f} mm")
        print(f"  cells changed by the correction  median {ch[len(ch)//2]:.2%}  "
              f"max {ch[-1]:.2%}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
