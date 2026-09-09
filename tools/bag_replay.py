#!/usr/bin/env python3
"""Replay bag-derived scenarios through the shield, in two frames.

Runs the shield twice over the same real local costmaps:

  local    the window's bottom-left corner is the world origin, which is
           how the committed scenarios are laid out and how the prompt
           describes the map to the model;
  shifted  the same grids placed at the robot-centred origin they
           actually have, with start, goal and every proposed waypoint
           moved by the same offset.

The decision must not depend on which frame the map is described in.
That is the property the committed scenarios cannot test, because all
forty of them sit at the origin.

The report also breaks outcomes down by unknown fraction, which is the
axis along which real costmaps differ most from synthetic ones.

Usage:
  python3 tools/bag_replay.py --scenarios ~/bagwork/scen \\
      --parsed ~/bagwork/llm_eval/parsed/qwen2.5-7b-instruct.txt \\
      --shield build/shield --out /tmp/bag_replay
"""
import argparse
import collections
import csv
import json
import pathlib
import shutil
import subprocess
import sys


def read_pgm_header(data):
    """Returns (header_len, width, height). Skips comment lines."""
    fields, i = [], 0
    while len(fields) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            i = data.index(b"\n", i) + 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    return i + 1, int(fields[1]), int(fields[2])


def shift_scenarios(src, dst, offset):
    """Copy a scenario directory with its world origin moved by offset."""
    (dst / "scenarios").mkdir(parents=True, exist_ok=True)
    ox, oy = offset
    for jf in sorted((src / "scenarios").glob("scenario_*.json")):
        scen = json.loads(jf.read_text())
        scen["origin"] = [ox, oy]
        scen["start"] = [scen["start"][0] + ox, scen["start"][1] + oy]
        scen["goal"] = [scen["goal"][0] + ox, scen["goal"][1] + oy]
        (dst / "scenarios" / jf.name).write_text(json.dumps(scen, indent=2))

        pf = jf.with_suffix(".pgm")
        raw = pf.read_bytes()
        hlen, w, h = read_pgm_header(raw)
        # The PGM comment takes precedence over the sidecar in the shield,
        # so the raster has to carry the shifted origin too.
        header = (f"P5\n# resolution {scen['resolution']:g} origin {ox:g} {oy:g}\n"
                  f"{w} {h}\n255\n").encode()
        (dst / "scenarios" / pf.name).write_bytes(header + raw[hlen:])


def shift_parsed(src_txt, dst_txt, offset):
    ox, oy = offset
    out, lines = [], src_txt.read_text().splitlines()
    i = 0
    while i < len(lines):
        parts = lines[i].split()
        if not parts:
            i += 1
            continue
        if parts[0] in ("scenario", "parse_ok"):
            out.append(lines[i])
        elif parts[0] in ("start", "goal"):
            out.append(f"{parts[0]} {float(parts[1]) + ox} {float(parts[2]) + oy}")
        elif parts[0] == "waypoints":
            n = int(parts[1])
            out.append(lines[i])
            for k in range(1, n + 1):
                x, y = lines[i + k].split()
                out.append(f"{float(x) + ox} {float(y) + oy}")
            i += n
        i += 1
    dst_txt.parent.mkdir(parents=True, exist_ok=True)
    dst_txt.write_text("\n".join(out) + "\n")


def run_shield(shield, parsed, scen_dir, out_csv):
    res = subprocess.run(
        [str(shield), "--parsed", str(parsed), "--scenarios", str(scen_dir),
         "--out", str(out_csv)],
        capture_output=True, text=True)
    return res.returncode, res.stdout, res.stderr


def buckets(csv_path, suite):
    rows = list(csv.DictReader(open(csv_path)))
    return {int(r["scenario"]): r["bucket"] for r in rows if r["suite"] == suite}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", required=True)
    ap.add_argument("--parsed", required=True)
    ap.add_argument("--shield", default="build/shield")
    ap.add_argument("--out", required=True)
    ap.add_argument("--offset", type=float, nargs=2, default=[-5.0, -5.0])
    args = ap.parse_args()

    src = pathlib.Path(args.scenarios).expanduser()
    parsed = pathlib.Path(args.parsed).expanduser()
    out = pathlib.Path(args.out).expanduser()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    print("=== local frame (window corner at the world origin) ===")
    rc_a, so_a, se_a = run_shield(args.shield, parsed, src / "scenarios",
                                  out / "local.csv")
    print(so_a.rstrip() or se_a.rstrip())

    print()
    print(f"=== shifted frame (origin {args.offset[0]:g}, {args.offset[1]:g}) ===")
    shift_scenarios(src, out / "shifted", args.offset)
    shift_parsed(parsed, out / "shifted" / "parsed.txt", args.offset)
    rc_b, so_b, se_b = run_shield(args.shield, out / "shifted" / "parsed.txt",
                                  out / "shifted" / "scenarios", out / "shifted.csv")
    print(so_b.rstrip() or se_b.rstrip())

    print()
    print("=== frame invariance ===")
    mismatch = 0
    for suite in ("qwen", "sealed_goal"):
        a, b = buckets(out / "local.csv", suite), buckets(out / "shifted.csv", suite)
        if not a and not b:
            continue
        for sid in sorted(set(a) | set(b)):
            if a.get(sid) != b.get(sid):
                mismatch += 1
                print(f"  {suite} scenario {sid}: {a.get(sid)} -> {b.get(sid)}")
    print(f"  {'MISMATCHES: %d' % mismatch if mismatch else 'every decision identical in both frames'}")

    print()
    print("=== outcome against how much of the map is unknown ===")
    index = {r["id"]: r for r in json.loads((src / "index.json").read_text())}
    local = buckets(out / "local.csv", "qwen")
    bands = [(0.0, 0.4), (0.4, 0.5), (0.5, 0.6), (0.6, 0.7), (0.7, 1.01)]
    tally = {b: collections.Counter() for b in bands}
    for sid, bucket in local.items():
        u = index[sid]["unknown_fraction"]
        for b in bands:
            if b[0] <= u < b[1]:
                tally[b][bucket] += 1
                break
    print(f"  {'unknown':>12}  {'n':>3}  outcomes")
    for b in bands:
        t = tally[b]
        if not sum(t.values()):
            continue
        desc = ", ".join(f"{k}={v}" for k, v in sorted(t.items()))
        print(f"  {b[0]:.0%}-{b[1]:.0%}".rjust(14) + f"  {sum(t.values()):>3}  {desc}")

    edges = [index[s]["free_edge_cells"] for s in local]
    print()
    print(f"free cells on the window edge: min {min(edges)}, median "
          f"{sorted(edges)[len(edges) // 2]}, max {max(edges)} of 800")
    print("(the committed scenarios have 0: every one is walled)")
    return 0 if rc_a == 0 and rc_b == 0 and mismatch == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
