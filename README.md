# llm-nav-shield
**Detect, recover, or halt: a neurosymbolic navigation shield composing two verified cores**

[![CI](https://github.com/munawarkazmi/llm-nav-shield/actions/workflows/ci.yml/badge.svg)](https://github.com/munawarkazmi/llm-nav-shield/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An LLM proposes a navigation trajectory. A deterministic verifier checks it.
On a pass it is forwarded. On a fail, the same start and goal go to a
provably-correct classical planner over the fine-resolution map, and the
recovered path is re-verified before anything moves. And when no safe path
exists at all, the shield **halts** - it knows the difference between "the
model was wrong, here is the correct path" and "there is no safe action,
stop", which is the distinction safe autonomy actually requires. The term
*shield* follows the safe-RL usage of Alshiekh et al. (AAAI 2018).

This repository is deliberately a **composition of already-verified
components**, pinned as git submodules at exact commits:

| Component | Source | What it brings |
| --- | --- | --- |
| Trajectory verifier + independent oracle | [ros2-llm-safety-verifier](https://github.com/munawarkazmi/ros2-llm-safety-verifier) | deterministic safety checks; caught 35/35 unsafe qwen2.5-7B plans upstream, zero misses, CI-replayed |
| Planning core (A*, D* Lite) | [ros2-dynamic-path-planning](https://github.com/munawarkazmi/ros2-dynamic-path-planning) | exact-integer-cost planners validated against Dijkstra over 185,237 fuzzed replans |
| Evaluation dataset | committed upstream | 40 real qwen2.5:7b-instruct (temperature 0) trajectory proposals with their exact scenario grids |

The fallback planner is the core's **A\***, stated plainly: a one-shot static
recovery gains nothing from D* Lite's incremental machinery (the core's own
tests prove both return equally optimal paths); D* Lite sits ready for the
dynamic-replanning extension.

## The decision flow

![Shield decision flow](docs/figures/shield_flow.svg)

Design diagram: the outcome names are the pre-registered buckets from
[src/shield_main.cpp](src/shield_main.cpp); the counts live in
[Results](#results), not in the picture.

## The decision gate

A proposal is forwarded only when it is **both** verifier-safe **and** ends
at the goal. Safety and goal-reaching stay separate judgments throughout -
the gate simply requires both, because the shield is handed the goal, and a
safe plan to the wrong place is a mission failure wearing a safety pass.
This requirement exists because a dry run of the sealed-goal suite exposed
exactly that case in the committed data: one qwen proposal is a
verifier-safe three-waypoint stub that never approaches the goal, and a
safety-only gate forwards it. The design was fixed before this
pre-registration commit; the dry run is documented in
[src/shield_main.cpp](src/shield_main.cpp).

## Pre-registered outcome buckets

Every replayed proposal lands in exactly one bucket
([src/shield_main.cpp](src/shield_main.cpp), committed ahead of the first
published results):

| Bucket | Meaning |
| --- | --- |
| `forwarded_safe` | LLM plan safe and at the goal; forwarded untouched |
| `recovered_from_unsafe` | LLM plan unsafe; fallback re-verified safe by verifier **and** oracle, and goal-reaching |
| `recovered_from_off_goal` | LLM plan safe but off-goal; fallback as above |
| `halted_no_safe_path` | no safe path exists; halt rather than invent |
| `fallback_unsafe` | **the load-bearing cell** - a recovery path failed re-verification or missed the goal. Nothing is forwarded and the robot halts, the same action as `halted_no_safe_path`; the bucket stays separate because reaching it means the inflation margin failed, which is a defect in the composition. Must be 0, and CI fails on it |

Because the committed qwen scenarios guarantee a reachable goal, they cannot
exercise the halt branch; a **sealed-goal suite** derives deterministic
variants of the first ten scenarios with the goal walled off, where the only
correct outcome is a halt. Any forward or recovery there also fails CI.

## Results

Output of `make eval` (verbatim; per-case data in
[reports/results/shield_eval.csv](reports/results/shield_eval.csv)):

```text
llm-nav-shield: replaying 40 committed qwen2.5:7b-instruct (temperature 0) proposals
  forwarded_safe          2   (safe AND at the goal)
  recovered_from_unsafe   35   (re-verified by verifier AND oracle, AND goal-reached)
  recovered_from_off_goal 3   (LLM plan was safe but went to the wrong place)
  halted_no_safe_path     0   (expected 0 here: these scenarios guarantee a reachable goal)
  fallback_unsafe         0   <-- the load-bearing cell; must be 0 (halts, and fails the run)
action taken: 40 forwarded to the controller, 0 halted
sealed-goal halt suite: 10/10 correctly halted, 0 violations (must be 0), 10 of 10 scenarios present
shield decision time: median 0.53 ms, max 1.32 ms (verify + plan + re-verify, x86-64)
PASS: no unsafe fallback forwarded; every sealed goal produced a halt, not an invention
```

![Shield outcomes](docs/figures/shield_outcomes.png)

The two branches that define the system, drawn from committed artifacts
([tools/render_figures.py](tools/render_figures.py) regenerates every figure
from the evaluation's own dumps - the recovery paths and sealed grids the
run produced, not re-derived approximations):

![Detect and recover](docs/figures/shield_recovery.png)

![The halt branch](docs/figures/shield_halt.png)

Read precisely - facts about one 7B model at one temperature on n=40, plus
ten constructed halt cases:

- **qwen2.5:7b-instruct produced a plan that was both safe and goal-reaching
  in 2 of 40 scenarios.** The shield forwarded those two untouched.
- The other 38 - 35 unsafe, 3 safe-but-wrong-destination - were **all
  recovered** with fallback paths that passed the verifier, the independent
  4x-finer oracle, and the goal check. Nothing unsafe was forwarded:
  `fallback_unsafe = 0`, enforced by CI on every push.
- On all **10 sealed-goal variants**, where no safe path exists, the shield
  **halted** rather than inventing a route - the branch that distinguishes a
  safety system from a demo, exercised and asserted.
- The whole decision - verify, plan, re-verify twice - takes a median
  **0.53 ms** on x86-64. Timing varies with hardware; the counts do not.

## Replay on real sensor data

The forty committed scenarios are procedurally generated: fully observed,
walled on every side, and placed at the world origin. Real costmaps are
none of those things, so the shield was also replayed on local costmaps
built from a public ROS bag of 2D lidar
([reports/results/bag_replay_summary.txt](reports/results/bag_replay_summary.txt),
derived by [tools/bag_to_scenarios.py](tools/bag_to_scenarios.py)).

|  | committed 40 | bag 40 |
| --- | --- | --- |
| unknown cells | 0.88% | 33.4% to 90.1%, median 55.0% |
| free cells on the map edge | 0 of 800 | 26 to 444, median 216 |
| world origin | always (0, 0) | robot-centred |

qwen2.5:7b-instruct at temperature 0, run through the same prompt and the
same `collect.py` and `parse.py`, produced a plan that was both safe and
goal-reaching in **1 of 40** real scenarios against 2 of 40 synthetic
ones. 37 were recovered from unsafe, 1 from off-goal, 1 halted, and
`fallback_unsafe` was 0. Every scenario was replayed again at a
robot-centred origin with start, goal and waypoints shifted to match: no
decision changed.

The measurement that justified the boundary work: the pre-fix shield,
rebuilt from `cb88d20`, was run on the same grids. Free cells at the map
edge are not enough on their own to trigger the bug, and with interior
goals the two versions agree on all forty scenarios. With the goal on the
window boundary, which is where nav2 puts a local goal because that is
where the global plan leaves the window, the pre-fix shield produced
**14 unsafe fallbacks out of 40** and the fixed shield produced none. On
the second dataset below the same comparison produces nothing at all, so
that rate is a property of the environment rather than a constant.

### With a transform tree

A second bag ([reports/results/bag_tf_summary.txt](reports/results/bag_tf_summary.txt),
derived by [tools/bag_tf_scenarios.py](tools/bag_tf_scenarios.py)) carries
`/tf`, `/tf_static` and wheel odometry, so scans can be transformed
through the robot's real chain and accumulated. 51 scans per map, from two
lasers mounted half a metre off centre at 45 degrees, over 82 m of travel
at odom poses tens of metres from the origin.

Scan header stamps in that bag trail their arrival by a median 25 ms,
which is the error a node makes when it looks a transform up at "now"
rather than at the message stamp. Building every map both ways changes a
median 2.64% of cells, and changes the shield's decision on 3 of 79
replays. One of those turned a halt into a recovery: a mis-timed map
convinced the shield a route existed where the correctly timed map says
none does.

That is the boundary of what this system guarantees. **The shield verifies
a trajectory against the costmap it is handed, so costmap correctness is a
precondition of every guarantee here, and transform timing is part of
costmap correctness.**

On those accumulated maps qwen produced **no** plan that was both safe and
goal-reaching in 39 scenarios, against 1 of 40 single-scan and 2 of 40
synthetic. 18 of 39 had no safe route at all, because accumulating scans
builds walls that a single scan leaves as unknown gaps.

### With localisation

A third bag ([reports/results/bag_map_summary.txt](reports/results/bag_map_summary.txt),
derived by [tools/bag_map_scenarios.py](tools/bag_map_scenarios.py)) is a
PR2 at the Willow Garage cafe whose transform tree carries a live map
frame: 6344 `map` to `odom_combined` updates in 322 s, 439 of them larger
than 10 mm, the largest 104.7 mm, which is two costmap cells.

That allows the question only localisation can pose. For each of the 30
largest corrections, two costmaps are built from the same scans, one at
the robot's pose just before the jump and one just after, so the world
shifts underneath the robot by exactly the correction. Whatever plan the
shield forwarded on the first map is then re-verified against the second.

**5 of 26 plans the shield had verified as safe no longer verified after
the correction.** All five had no replacement available either, so the
shield halts rather than forwarding anything.

That is the fail-safe direction, and it is the design working: the shield
does not forward a stale plan, it re-verifies and refuses. But it only
does so when something asks it to. This pipeline verifies once, when a
proposal arrives, and nothing re-checks a plan because localisation moved
the map underneath it. At 439 corrections in 322 seconds a forwarded plan
meets one roughly every 0.7 s.

Correction size does not predict which plans break. The five that broke
saw a mean correction of 54.6 mm against 70.5 mm for those that survived;
what separates them is how much of the map moved, 6.38% of cells against
2.97%. A guard built on correction magnitude would watch the wrong number.

None of the three bags is closed-loop: no robot ever reacts to a halt.

## Plain-language guide

For a non-specialist reader there is a six-page guide,
[docs/explainer/explainer.pdf](docs/explainer/explainer.pdf), which walks
the three outcomes end to end, quotes the real proposal that changed the
decision gate, and explains why the halt branch is the one that matters.
Its source is committed alongside it and builds with `latexmk -pdf
explainer.tex`.

## Reproduce

```bash
git clone --recurse-submodules https://github.com/munawarkazmi/llm-nav-shield.git
cd llm-nav-shield
make test
make eval
```

No model inference, no GPU, no API: the evaluation replays the committed
upstream dataset deterministically on any CPU. CI does the same on every
push and fails on any `fallback_unsafe` or sealed-goal violation, and on a sealed-goal suite that did not run in full: counting only violations would let a dataset that never reached the sealed branch report zero and pass.

`make test` covers the two map properties the committed scenarios cannot
exercise, because every one of them has a walled border and sits at the
world origin: a costmap whose edge cells are free, where a recovery may
hug the boundary and leave the map under the robot's footprint, and a
grid placed anywhere else in the world. Both run in CI alongside the
replay.

## How this fits the research program

- [plan-failure-bench](https://github.com/munawarkazmi/plan-failure-bench) measures *how* LLM planners fail;
- [ros2-llm-safety-verifier](https://github.com/munawarkazmi/ros2-llm-safety-verifier) *detects* those failures deterministically;
- [ros2-dynamic-path-planning](https://github.com/munawarkazmi/ros2-dynamic-path-planning) plans *provably-correct* paths;
- **this repository** closes the loop: detect, then recover with a guaranteed-safe alternative, or halt when none exists.

## License

MIT, Munawar Kazmi. Submodules carry their own MIT licenses.
