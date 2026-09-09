#!/usr/bin/env python3
"""Turn real lidar scans from a ROS bag into shield evaluation scenarios.

The committed forty scenarios are synthetic: procedurally generated rooms
that are fully observed, walled on every side, and placed at the world
origin. Real costmaps are none of those things. This script builds
scenarios from a public bag so the shield meets the properties the
synthetic set cannot produce.

Each selected scan becomes one robot-centred local costmap, which is what
nav2's local costmap actually is: a rolling window rebuilt from recent
sensor data. Rays are cast from the sensor to each return, clearing the
cells they cross and marking the return lethal. Cells no ray reaches stay
unknown, so sensor shadows and the blind sector behind the scanner are
unknown for the real reason rather than by construction. Cells at the
window edge are free wherever a ray left through them.

The output matches the format of the committed scenarios, so
llm_eval/collect.py and llm_eval/parse.py apply unchanged, with one
addition: an "origin" field, which the shield reads to place the grid in
the world.

Data (not committed, ~90 MB, fetched on demand):
  https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/
  b2-2016-04-05-14-44-52.bag
  sha256 095bd3b04799a5f69158faa6c6fe3ef4f1e4b7a0b02e91e85344b0d4787f1bcb

Requires: pip install rosbags numpy

Usage:
  python3 tools/bag_to_scenarios.py --bag b2.bag --out /tmp/bag_scenarios \\
      --count 40
"""
import argparse
import json
import math
import pathlib
import sys

import numpy as np
from rosbags.highlevel import AnyReader

FREE, LETHAL, UNKNOWN = 0, 254, 255


def first_echo_ranges(msg):
    """MultiEchoLaserScan carries a LaserEcho per beam; take the first."""
    out = []
    for echo in msg.ranges:
        e = list(echo.echoes)
        out.append(e[0] if e else math.inf)
    return out


def build_local_costmap(msg, cells, res, robot_radius=0.105):
    """One scan into a robot-centred grid. Returns a (cells, cells) array.

    Index [y][x] with y = 0 at the bottom, matching verifier::Grid, whose
    cellCenter treats row 0 as the lowest y.

    The robot's own footprint is cleared, as nav2's costmap does. This
    scanner covers 269.5 degrees, so roughly ninety degrees behind it are
    unobserved, and without clearing the footprint those unknown cells sit
    against the robot and no plan can start at all.
    """
    grid = np.full((cells, cells), UNKNOWN, dtype=np.uint8)
    c = cells // 2  # sensor cell
    ranges = first_echo_ranges(msg)
    max_reach = (cells / 2.0) * res * 1.5  # beyond the window corner

    for i, r in enumerate(ranges):
        ang = msg.angle_min + i * msg.angle_increment
        hit = math.isfinite(r) and msg.range_min <= r <= msg.range_max
        # Clear along the ray; a beam with no return still clears out to
        # the window edge, which is how a costmap treats max-range beams.
        reach = r if hit else max_reach
        steps = int(reach / (res * 0.5)) + 1
        dx, dy = math.cos(ang), math.sin(ang)
        last = None
        for s in range(steps + 1):
            d = s * res * 0.5
            if d > reach:
                break
            x = int(math.floor(c + d * dx / res))
            y = int(math.floor(c + d * dy / res))
            if not (0 <= x < cells and 0 <= y < cells):
                break
            if (x, y) != last:
                if grid[y][x] == UNKNOWN:
                    grid[y][x] = FREE
                last = (x, y)
        if hit:
            x = int(math.floor(c + r * dx / res))
            y = int(math.floor(c + r * dy / res))
            if 0 <= x < cells and 0 <= y < cells:
                grid[y][x] = LETHAL

    # Footprint clearing, plus a cell of margin so the start has room.
    fp = int(math.ceil((robot_radius + res) / res))
    for dy in range(-fp, fp + 1):
        for dx in range(-fp, fp + 1):
            if dx * dx + dy * dy > fp * fp:
                continue
            x, y = c + dx, c + dy
            if 0 <= x < cells and 0 <= y < cells and grid[y][x] != LETHAL:
                grid[y][x] = FREE
    return grid


def to_ascii(grid, block):
    """Downsample for the prompt. Conservative: a block containing any
    lethal cell is an obstacle, any unknown cell is unmapped."""
    cells = grid.shape[0]
    rows = cells // block
    out = []
    for r in range(rows):
        y0 = cells - (r + 1) * block  # ascii row 0 is the highest y band
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


def clearance_ok(grid, x, y, radius_cells):
    """True when every cell within radius_cells is free and on the map."""
    cells = grid.shape[0]
    r = int(math.ceil(radius_cells))
    if x - r < 0 or y - r < 0 or x + r >= cells or y + r >= cells:
        return False
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            if grid[y + dy][x + dx] != FREE:
                return False
    return True


def pick_goal(grid, sx, sy, radius_cells):
    """Farthest cell from the start that the planner would call traversable.

    radius_cells is the shield's own inflation radius, not the bare robot
    radius, so the goal is a cell the planner could actually occupy. A
    goal inside the inflated region would make every scenario halt for a
    trivial reason and hide whether a route exists at all.
    """
    cells = grid.shape[0]
    best, best_d = None, -1.0
    for y in range(cells):
        for x in range(cells):
            if grid[y][x] != FREE:
                continue
            d = (x - sx) ** 2 + (y - sy) ** 2
            if d <= best_d:
                continue
            if clearance_ok(grid, x, y, radius_cells):
                best, best_d = (x, y), d
    return best


def pick_edge_goal(grid, sx, sy, band=3):
    """Farthest free cell lying in the outer band of the window.

    This is where a local planner's goal usually is. nav2 hands the local
    controller the point at which the global plan leaves the local
    costmap, which is on the window boundary by definition. The interior
    policy above is the gentler case; this one is the common one.
    """
    cells = grid.shape[0]
    best, best_d = None, -1.0
    for y in range(cells):
        for x in range(cells):
            on_band = x < band or y < band or x >= cells - band or y >= cells - band
            if not on_band or grid[y][x] != FREE:
                continue
            d = (x - sx) ** 2 + (y - sy) ** 2
            if d > best_d:
                best, best_d = (x, y), d
    return best


def write_pgm(path, grid, res, origin):
    cells = grid.shape[0]
    with open(path, "wb") as f:
        f.write(f"P5\n# resolution {res:g} origin {origin[0]:g} {origin[1]:g}\n"
                f"{cells} {cells}\n255\n".encode())
        f.write(grid.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="horizontal_laser_2d")
    ap.add_argument("--out", required=True)
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--cells", type=int, default=200, help="window is cells x cells")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--ascii-block", type=int, default=5, help="cells per ascii char")
    ap.add_argument("--robot-radius", type=float, default=0.105)
    ap.add_argument("--skip-start", type=int, default=200,
                    help="scans to skip before sampling, past the standing start")
    ap.add_argument("--goal-policy", choices=["interior", "edge"], default="interior",
                    help="interior: farthest cell the planner would accept. "
                         "edge: farthest free cell on the window boundary, which "
                         "is where a local planner's goal normally sits.")
    args = ap.parse_args()

    out = pathlib.Path(args.out)
    (out / "scenarios").mkdir(parents=True, exist_ok=True)

    bag = pathlib.Path(args.bag)
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            print(f"topic {args.topic} not in bag", file=sys.stderr)
            return 2
        total = sum(c.msgcount for c in conns)
        usable = total - args.skip_start
        stride = max(1, usable // args.count)
        print(f"{total} scans on {args.topic}; sampling every {stride} "
              f"from #{args.skip_start} for {args.count} scenarios")

        half = (args.cells / 2.0) * args.resolution
        origin = (-half, -half)          # the window is centred on the robot
        # The shield's inflation: footprint plus one cell, the same figure
        # inflatedPlanningGrid computes. Start and goal are held to it so
        # both are cells the planner would accept.
        radius_cells = float(math.ceil(
            (args.robot_radius + args.resolution) / args.resolution))
        extent = args.cells * args.resolution

        made, idx, index = 0, -1, []
        for conn, _ts, raw in reader.messages(connections=conns):
            idx += 1
            if idx < args.skip_start or (idx - args.skip_start) % stride:
                continue
            if made >= args.count:
                break
            msg = reader.deserialize(raw, conn.msgtype)
            grid = build_local_costmap(msg, args.cells, args.resolution,
                                       args.robot_radius)

            c = args.cells // 2
            if not clearance_ok(grid, c, c, radius_cells):
                continue  # sensor cell not clear: nothing sensible to plan from
            if args.goal_policy == "edge":
                goal = pick_edge_goal(grid, c, c)
            else:
                goal = pick_goal(grid, c, c, radius_cells)
            if goal is None:
                continue

            # Coordinates the model sees: bottom-left of the window is
            # (0, 0), exactly as the committed prompt states.
            start_m = ((c + 0.5) * args.resolution, (c + 0.5) * args.resolution)
            goal_m = ((goal[0] + 0.5) * args.resolution,
                      (goal[1] + 0.5) * args.resolution)

            unknown_frac = float((grid == UNKNOWN).mean())
            edge = np.concatenate([grid[0, :], grid[-1, :], grid[:, 0], grid[:, -1]])
            scen = {
                "id": made,
                "width": args.cells,
                "height": args.cells,
                "resolution": args.resolution,
                "origin": [0.0, 0.0],
                "robot_origin": [origin[0], origin[1]],
                "ascii_cell_m": args.resolution * args.ascii_block,
                "start": [round(start_m[0], 3), round(start_m[1], 3)],
                "goal": [round(goal_m[0], 3), round(goal_m[1], 3)],
                "scan_index": idx,
                "goal_policy": args.goal_policy,
                "unknown_fraction": round(unknown_frac, 4),
                "free_edge_cells": int((edge == FREE).sum()),
                "ascii": to_ascii(grid, args.ascii_block),
            }
            name = out / "scenarios" / f"scenario_{made:02d}"
            name.with_suffix(".json").write_text(json.dumps(scen, indent=2))
            write_pgm(name.with_suffix(".pgm"), grid, args.resolution, (0.0, 0.0))
            index.append({"id": made, "scan_index": idx,
                          "unknown_fraction": unknown_frac,
                          "free_edge_cells": int((edge == FREE).sum())})
            made += 1

    (out / "index.json").write_text(json.dumps(index, indent=2))
    if index:
        u = [r["unknown_fraction"] for r in index]
        e = [r["free_edge_cells"] for r in index]
        print(f"wrote {made} scenarios to {out}")
        print(f"  window            {args.cells}x{args.cells} cells "
              f"= {extent:g} m x {extent:g} m at {args.resolution} m")
        print(f"  unknown fraction  min {min(u):.1%}  median "
              f"{sorted(u)[len(u) // 2]:.1%}  max {max(u):.1%}")
        print(f"  free edge cells   min {min(e)}  median {sorted(e)[len(e) // 2]}"
              f"  max {max(e)}   (0 would mean a walled border)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
