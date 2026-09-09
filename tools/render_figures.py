#!/usr/bin/env python3
"""Render the README figures from committed shield-evaluation artifacts.

Inputs (all committed):
  reports/results/shield_eval.csv       outcome per case, both suites
  reports/results/dumps/fallback_*.txt  recovery trajectories the shield produced
  reports/results/dumps/sealed_*.pgm    sealed-goal variant grids
  deps/verifier/llm_eval/...            scenario grids + qwen trajectories

Usage: render_figures.py [outdir]
"""
import csv
import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEPS = ROOT / "deps/verifier/llm_eval"
DUMPS = ROOT / "reports/results/dumps"

NAVY = "#123a5f"
GREEN = "#2e7d4f"
GOLD = "#d9a441"
RED = "#b03a2e"
GRAY = "#8a8f98"

MODEL_LABEL = "qwen2.5:7b-instruct (temperature 0)"


def read_cost_pgm(path):
    """Reads a P5 grid, honouring comment lines.

    A comment of the form "# resolution R origin X Y" carries the grid's
    place in the world, which the raster itself does not record. Grids
    without one are 0.05 m cells at the world origin, as the committed
    scenarios are.
    """
    data = path.read_bytes()
    fields, res, origin, i = [], 0.05, (0.0, 0.0), 0
    while len(fields) < 4:
        while data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            end = data.index(b"\n", i)
            parts = data[i + 1 : end].split()
            for k, tok in enumerate(parts):
                if tok == b"resolution" and k + 1 < len(parts):
                    res = float(parts[k + 1])
                elif tok == b"origin" and k + 2 < len(parts):
                    origin = (float(parts[k + 1]), float(parts[k + 2]))
            i = end + 1
            continue
        j = i
        while not data[j : j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    i += 1
    w, h = int(fields[1]), int(fields[2])
    return w, h, data[i : i + w * h], res, origin


def map_image(path):
    w, h, cells, res, origin = read_cost_pgm(path)
    img = [[0.15 if cells[y * w + x] == 254 else (0.6 if cells[y * w + x] == 255 else 1.0)
            for x in range(w)] for y in range(h)]
    return w, h, img, res, origin


def scenario(sid):
    return json.loads((DEPS / "scenarios" / f"scenario_{sid:02d}.json").read_text())


def qwen_traj(sid):
    parsed = json.loads((DEPS / "parsed/qwen2.5-7b-instruct.json").read_text())
    return next(r["waypoints"] for r in parsed if r["scenario"] == sid)


def draw_map(ax, w, h, img, res=0.05, origin=(0.0, 0.0)):
    ax.imshow(img, cmap="gray", origin="lower", vmin=0, vmax=1,
              extent=[origin[0], origin[0] + w * res,
                      origin[1], origin[1] + h * res],
              interpolation="nearest")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def fig_outcomes(out):
    rows = list(csv.DictReader((ROOT / "reports/results/shield_eval.csv").open()))
    buckets = ["forwarded_safe", "recovered_from_unsafe", "recovered_from_off_goal",
               "halted_no_safe_path", "fallback_unsafe"]
    qwen = {b: sum(1 for r in rows if r["suite"] == "qwen" and r["bucket"] == b)
            for b in buckets}
    sealed = {b: sum(1 for r in rows if r["suite"] == "sealed_goal" and r["bucket"] == b)
              for b in buckets}

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.6),
                                   gridspec_kw={"width_ratios": [3, 2]})
    colors = [GREEN, NAVY, GOLD, GRAY, RED]
    bars = ax1.barh(range(len(buckets)), [qwen[b] for b in buckets], color=colors)
    ax1.set_yticks(range(len(buckets)), buckets)
    ax1.invert_yaxis()
    ax1.bar_label(bars, padding=3, fontsize=10)
    ax1.set_xlim(0, 40)
    ax1.set_xlabel("plans")
    ax1.set_title(f"All 40 {MODEL_LABEL} proposals\n"
                  "(fallback_unsafe is the load-bearing zero)", fontsize=10)

    bars2 = ax2.barh(range(len(buckets)), [sealed[b] for b in buckets], color=colors)
    ax2.set_yticks(range(len(buckets)), ["" for _ in buckets])
    ax2.invert_yaxis()
    ax2.bar_label(bars2, padding=3, fontsize=10)
    ax2.set_xlim(0, 11)
    ax2.set_xlabel("sealed-goal variants")
    ax2.set_title("Halt suite: goal walled off,\nonly correct outcome is a halt",
                  fontsize=10)

    fig.suptitle("Shield outcomes, replayed from committed data - "
                 "nothing unsafe forwarded, no route invented", fontsize=12, color=NAVY)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_recovery(out):
    # First recovered_from_unsafe scenario with a committed fallback dump.
    rows = list(csv.DictReader((ROOT / "reports/results/shield_eval.csv").open()))
    sid = next(int(r["scenario"]) for r in rows
               if r["suite"] == "qwen" and r["bucket"] == "recovered_from_unsafe")
    scen = scenario(sid)
    w, h, img, res, origin = map_image(DEPS / "scenarios" / f"scenario_{sid:02d}.pgm")
    llm = qwen_traj(sid)
    fb = [tuple(map(float, ln.split()))
          for ln in (DUMPS / f"fallback_{sid:02d}.txt").read_text().splitlines() if ln]

    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    draw_map(ax, w, h, img, res, origin)
    ax.plot([p[0] for p in llm], [p[1] for p in llm], c=RED, lw=2, ls="--",
            marker="o", ms=3, label="qwen2.5-7B proposal (rejected: unsafe)")
    ax.plot([p[0] for p in fb], [p[1] for p in fb], c=GREEN, lw=2.2,
            label="shield recovery (verifier + oracle + goal-checked)")
    ax.scatter([scen["start"][0]], [scen["start"][1]], c=GOLD, s=110, zorder=5,
               label="start")
    ax.scatter([scen["goal"][0]], [scen["goal"][1]], c=NAVY, s=110, zorder=5,
               label="goal")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(f"Detect and recover, scenario {sid}: the model's plan is rejected, "
                 "a provably-safe path is\nplanned on the fine grid, re-verified, and "
                 "forwarded instead (committed data)", fontsize=10, color=NAVY)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_halt(out):
    # Scenario 9: the safe-but-goes-nowhere stub, on its sealed variant.
    sid = 9
    scen = scenario(sid)
    w, h, img, res, origin = map_image(DUMPS / f"sealed_{sid:02d}.pgm")
    llm = qwen_traj(sid)

    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    draw_map(ax, w, h, img, res, origin)
    ax.plot([p[0] for p in llm], [p[1] for p in llm], c=RED, lw=2.5, ls="--",
            marker="o", ms=5, label="qwen2.5-7B proposal (safe but never nears the goal)")
    ax.scatter([scen["start"][0]], [scen["start"][1]], c=GOLD, s=110, zorder=5,
               label="start")
    ax.scatter([scen["goal"][0]], [scen["goal"][1]], c=NAVY, s=110, zorder=5,
               label="goal (sealed)")
    ax.annotate("no safe path exists\n-> shield HALTS",
                xy=(scen["goal"][0], scen["goal"][1]),
                xytext=(scen["goal"][0] - 4.0, scen["goal"][1] + 1.6),
                color=RED, fontsize=11, fontweight="bold",
                arrowprops={"color": RED, "arrowstyle": "->"})
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(f"The halt branch, scenario {sid} (sealed variant): the model's plan "
                 "is safe but goes nowhere,\nno safe route to the goal exists - the "
                 "shield halts instead of forwarding or inventing", fontsize=10,
                 color=NAVY)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    outdir = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs/figures"
    outdir.mkdir(parents=True, exist_ok=True)
    fig_outcomes(outdir / "shield_outcomes.png")
    fig_recovery(outdir / "shield_recovery.png")
    fig_halt(outdir / "shield_halt.png")
    print("figures written to", outdir)


if __name__ == "__main__":
    main()
