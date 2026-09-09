#!/usr/bin/env python3
"""Build shield scenarios from a ROS bag that carries TF and odometry.

tools/bag_to_scenarios.py works from a single scan because its bag has no
poses. This one uses the transform tree, which changes what can be built
and what can be tested:

  - scans are transformed through the real chain
    odom -> base_footprint -> base_link -> robot_center -> laser,
    so the lasers' mounting offsets and yaw actually matter;
  - several scans from both lasers are accumulated into one window, so
    the map carries real odometry drift rather than being a single
    instantaneous view;
  - the window sits at the robot's real odom pose, which in this dataset
    is tens of metres from the origin and negative.

It also makes the transform timing question measurable. Scan header
stamps in this bag trail their arrival by a median 25 ms. --stamp-policy
selects which time the pose lookup uses:

  header    the scan's own stamp, interpolated. Correct.
  receipt   the time the message was recorded. Wrong by that lag, which
            is the mistake a node makes when it uses "now" instead of the
            message stamp.

Requires: pip install rosbags numpy

Usage:
  python3 tools/bag_tf_scenarios.py --bag toru.bag --out /tmp/tf_scen \\
      --count 40 --stamp-policy header
"""
import argparse
import bisect
import json
import math
import pathlib
import sys

import numpy as np
from rosbags.highlevel import AnyReader

FREE, LETHAL, UNKNOWN = 0, 254, 255


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """2D rigid transforms as (x, y, yaw): a then b."""
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at),
            ay + bx * math.sin(at) + by * math.cos(at),
             at + bt)


def invert(t):
    x, y, th = t
    c, s = math.cos(-th), math.sin(-th)
    return (-(x * c - y * s), -(x * s + y * c), -th)


def static_chain(statics, target, root="base_footprint"):
    """Compose the fixed transforms from root down to target."""
    parent = {child: (par, tf) for (par, child), tf in statics.items()}
    chain, node = [], target
    while node != root:
        if node not in parent:
            return None
        par, tf = parent[node]
        chain.append(tf)
        node = par
    out = (0.0, 0.0, 0.0)
    for tf in reversed(chain):
        out = compose(out, tf)
    return out


class PoseTrack:
    """odom -> base_footprint over time, linearly interpolated."""

    def __init__(self):
        self.t, self.x, self.y, self.th = [], [], [], []

    def add(self, t, x, y, th):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.th.append(th)

    def at(self, t):
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i == 0:
            return (self.x[0], self.y[0], self.th[0])
        if i >= len(self.t):
            return (self.x[-1], self.y[-1], self.th[-1])
        t0, t1 = self.t[i - 1], self.t[i]
        f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        dth = (self.th[i] - self.th[i - 1] + math.pi) % (2 * math.pi) - math.pi
        return (self.x[i - 1] + f * (self.x[i] - self.x[i - 1]),
                self.y[i - 1] + f * (self.y[i] - self.y[i - 1]),
                self.th[i - 1] + f * dth)


def integrate_scan(hits, passes, scan, laser_pose_grid, cells, res, max_reach):
    """Mark one scan into hit and pass counters, in grid cells."""
    lx, ly, lth = laser_pose_grid
    n = len(scan["ranges"])
    ang = scan["angle_min"] + np.arange(n) * scan["angle_increment"] + lth
    rng = np.asarray(scan["ranges"], dtype=np.float64)
    good = np.isfinite(rng) & (rng >= scan["range_min"]) & (rng <= scan["range_max"])
    reach = np.where(good, rng, max_reach)
    reach = np.minimum(reach, max_reach)

    step = res * 0.5
    smax = int(max_reach / step) + 1
    d = np.arange(smax) * step
    on = d[None, :] <= reach[:, None]
    xs = lx + d[None, :] * np.cos(ang)[:, None]
    ys = ly + d[None, :] * np.sin(ang)[:, None]
    cx = np.floor(xs / res).astype(np.int64)
    cy = np.floor(ys / res).astype(np.int64)
    ok = on & (cx >= 0) & (cx < cells) & (cy >= 0) & (cy < cells)
    np.add.at(passes, (cy[ok], cx[ok]), 1)

    hx = lx + rng[good] * np.cos(ang[good])
    hy = ly + rng[good] * np.sin(ang[good])
    hcx = np.floor(hx / res).astype(np.int64)
    hcy = np.floor(hy / res).astype(np.int64)
    hok = (hcx >= 0) & (hcx < cells) & (hcy >= 0) & (hcy < cells)
    np.add.at(hits, (hcy[hok], hcx[hok]), 1)


def to_ascii(grid, block):
    cells = grid.shape[0]
    rows = cells // block
    out = []
    for r in range(rows):
        y0 = cells - (r + 1) * block
        line = []
        for cx in range(rows):
            b = grid[y0:y0 + block, cx * block:(cx + 1) * block]
            if (b == LETHAL).any():
                line.append("#")
            elif (b == UNKNOWN).any():
                line.append("?")
            else:
                line.append(".")
        out.append("".join(line))
    return out


def clearance_ok(grid, x, y, r_cells):
    cells = grid.shape[0]
    r = int(math.ceil(r_cells))
    if x - r < 0 or y - r < 0 or x + r >= cells or y + r >= cells:
        return False
    sub = grid[y - r:y + r + 1, x - r:x + r + 1]
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    mask = (xx * xx + yy * yy) <= r_cells * r_cells
    return bool((sub[mask] == FREE).all())


def pick_goal(grid, sx, sy, r_cells):
    cells = grid.shape[0]
    ys, xs = np.nonzero(grid == FREE)
    d = (xs - sx) ** 2 + (ys - sy) ** 2
    for i in np.argsort(-d):
        x, y = int(xs[i]), int(ys[i])
        if clearance_ok(grid, x, y, r_cells):
            return (x, y)
    return None


def pick_edge_goal(grid, sx, sy, band=3):
    """Farthest free cell in the outer band, where a local planner's goal
    normally sits: the point at which the global plan leaves the window."""
    cells = grid.shape[0]
    ys, xs = np.nonzero(grid == FREE)
    on = (xs < band) | (ys < band) | (xs >= cells - band) | (ys >= cells - band)
    xs, ys = xs[on], ys[on]
    if not len(xs):
        return None
    i = int(np.argmax((xs - sx) ** 2 + (ys - sy) ** 2))
    return (int(xs[i]), int(ys[i]))


def write_pgm(path, grid, res, origin):
    cells = grid.shape[0]
    with open(path, "wb") as f:
        f.write(f"P5\n# resolution {res:g} origin {origin[0]:g} {origin[1]:g}\n"
                f"{cells} {cells}\n255\n".encode())
        f.write(grid.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scan-topics", nargs="+", default=["/scan_front", "/scan_rear"])
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--cells", type=int, default=200)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--ascii-block", type=int, default=5)
    ap.add_argument("--robot-radius", type=float, default=0.105)
    ap.add_argument("--window-s", type=float, default=2.0,
                    help="seconds of scans accumulated into each map")
    ap.add_argument("--stamp-policy", choices=["header", "receipt"], default="header")
    ap.add_argument("--goal-policy", choices=["interior", "edge"], default="interior")
    args = ap.parse_args()

    out = pathlib.Path(args.out)
    (out / "scenarios").mkdir(parents=True, exist_ok=True)
    res, cells = args.resolution, args.cells
    half = cells / 2.0 * res
    infl = float(math.ceil((args.robot_radius + res) / res))
    max_reach = half * 1.45

    with AnyReader([pathlib.Path(args.bag)]) as r:
        statics = {}
        for c in [c for c in r.connections if c.topic == "/tf_static"]:
            for _c, _t, raw in r.messages(connections=[c]):
                for tr in r.deserialize(raw, _c.msgtype).transforms:
                    a = tr.header.frame_id.lstrip("/")
                    b = tr.child_frame_id.lstrip("/")
                    statics[(a, b)] = (tr.transform.translation.x,
                                       tr.transform.translation.y,
                                       yaw_of(tr.transform.rotation))
        print("static transforms:", len(statics))

        track = PoseTrack()
        oc = [c for c in r.connections if c.topic == args.odom_topic]
        for conn, ts, raw in r.messages(connections=oc):
            m = r.deserialize(raw, conn.msgtype)
            s = m.header.stamp
            track.add(s.sec + s.nanosec * 1e-9, m.pose.pose.position.x,
                      m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        print(f"odom poses: {len(track.t)} spanning {track.t[-1] - track.t[0]:.1f} s")

        scans, lag = [], []
        sc = [c for c in r.connections if c.topic in args.scan_topics]
        for conn, ts, raw in r.messages(connections=sc):
            m = r.deserialize(raw, conn.msgtype)
            hs = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            lag.append(ts / 1e9 - hs)
            frame = m.header.frame_id.lstrip("/")
            tf = static_chain(statics, frame)
            if tf is None:
                continue
            scans.append({
                "t_header": hs, "t_receipt": ts / 1e9, "tf": tf,
                "angle_min": m.angle_min, "angle_increment": m.angle_increment,
                "range_min": m.range_min, "range_max": m.range_max,
                "ranges": np.asarray(m.ranges, dtype=np.float64),
            })
        scans.sort(key=lambda s: s["t_header"])
        lag.sort()
        print(f"scans: {len(scans)}, header-to-receipt lag median "
              f"{lag[len(lag) // 2] * 1000:.1f} ms")

    key = "t_header" if args.stamp_policy == "header" else "t_receipt"
    times = [s[key] for s in scans]
    t0, t1 = times[0] + args.window_s, times[-1]
    keyframes = [t0 + i * (t1 - t0) / max(1, args.count - 1) for i in range(args.count)]

    made, index = 0, []
    for kt in keyframes:
        kp = track.at(kt)
        if kp is None:
            continue
        inv_kp = invert(kp)
        hits = np.zeros((cells, cells), dtype=np.int32)
        passes = np.zeros((cells, cells), dtype=np.int32)
        lo = bisect.bisect_left(times, kt - args.window_s)
        hi = bisect.bisect_right(times, kt)
        used = 0
        for s in scans[lo:hi]:
            pose = track.at(s[key])
            if pose is None:
                continue
            # laser in odom, then into the keyframe's robot frame, then
            # into grid coordinates whose corner is the window corner.
            laser_odom = compose(pose, s["tf"])
            rel = compose(inv_kp, laser_odom)
            grid_pose = (rel[0] + half, rel[1] + half, rel[2])
            integrate_scan(hits, passes, s, grid_pose, cells, res, max_reach)
            used += 1

        grid = np.full((cells, cells), UNKNOWN, dtype=np.uint8)
        grid[passes > 0] = FREE
        grid[hits > 0] = LETHAL
        fp = int(math.ceil((args.robot_radius + res) / res))
        c = cells // 2
        yy, xx = np.ogrid[-fp:fp + 1, -fp:fp + 1]
        disc = (xx * xx + yy * yy) <= fp * fp
        # Unconditional, as nav2's footprint clearing is: with two lasers
        # mounted half a metre apart each sees part of the robot's own
        # body, and those returns would otherwise wall the robot in.
        blk = grid[c - fp:c + fp + 1, c - fp:c + fp + 1]
        blk[disc] = FREE

        if not clearance_ok(grid, c, c, infl):
            continue
        goal = (pick_edge_goal(grid, c, c) if args.goal_policy == "edge"
                else pick_goal(grid, c, c, infl))
        if goal is None:
            continue

        start_m = ((c + 0.5) * res, (c + 0.5) * res)
        goal_m = ((goal[0] + 0.5) * res, (goal[1] + 0.5) * res)
        unknown = float((grid == UNKNOWN).mean())
        edge = np.concatenate([grid[0, :], grid[-1, :], grid[:, 0], grid[:, -1]])
        scen = {
            "id": made, "width": cells, "height": cells, "resolution": res,
            "origin": [0.0, 0.0],
            "odom_pose": [round(kp[0], 3), round(kp[1], 3), round(kp[2], 4)],
            "ascii_cell_m": res * args.ascii_block,
            "start": [round(start_m[0], 3), round(start_m[1], 3)],
            "goal": [round(goal_m[0], 3), round(goal_m[1], 3)],
            "scans_accumulated": used,
            "stamp_policy": args.stamp_policy,
            "goal_policy": args.goal_policy,
            "unknown_fraction": round(unknown, 4),
            "free_edge_cells": int((edge == FREE).sum()),
            "ascii": to_ascii(grid, args.ascii_block),
        }
        base = out / "scenarios" / f"scenario_{made:02d}"
        base.with_suffix(".json").write_text(json.dumps(scen, indent=2))
        write_pgm(base.with_suffix(".pgm"), grid, res, (0.0, 0.0))
        index.append({"id": made, "unknown_fraction": unknown,
                      "free_edge_cells": int((edge == FREE).sum()),
                      "scans_accumulated": used,
                      "odom_pose": scen["odom_pose"]})
        made += 1

    (out / "index.json").write_text(json.dumps(index, indent=2))
    if index:
        u = sorted(r["unknown_fraction"] for r in index)
        e = sorted(r["free_edge_cells"] for r in index)
        a = sorted(r["scans_accumulated"] for r in index)
        print(f"wrote {made} scenarios to {out}  (stamp policy: {args.stamp_policy})")
        print(f"  scans per map     min {a[0]}  median {a[len(a) // 2]}  max {a[-1]}")
        print(f"  unknown fraction  min {u[0]:.1%}  median {u[len(u) // 2]:.1%}  "
              f"max {u[-1]:.1%}")
        print(f"  free edge cells   min {e[0]}  median {e[len(e) // 2]}  max {e[-1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
