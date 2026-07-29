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
| `fallback_unsafe` | **the load-bearing cell** - a recovery path failed re-verification or missed the goal; must be 0, and CI fails on it |

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
  fallback_unsafe         0   <-- the load-bearing cell; must be 0
sealed-goal halt suite: 10/10 correctly halted, 0 violations (must be 0)
shield decision time: median 0.48 ms, max 1.40 ms (verify + plan + re-verify, x86-64)
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
  **0.48 ms** on x86-64. Timing varies with hardware; the counts do not.

## Reproduce

```bash
git clone --recurse-submodules https://github.com/munawarkazmi/llm-nav-shield.git
cd llm-nav-shield
make eval
```

No model inference, no GPU, no API: the evaluation replays the committed
upstream dataset deterministically on any CPU. CI does the same on every
push and fails on any `fallback_unsafe` or sealed-goal violation.

## How this fits the research program

- [plan-failure-bench](https://github.com/munawarkazmi/plan-failure-bench) measures *how* LLM planners fail;
- [ros2-llm-safety-verifier](https://github.com/munawarkazmi/ros2-llm-safety-verifier) *detects* those failures deterministically;
- [ros2-dynamic-path-planning](https://github.com/munawarkazmi/ros2-dynamic-path-planning) plans *provably-correct* paths;
- **this repository** closes the loop: detect, then recover with a guaranteed-safe alternative, or halt when none exists.

## License

MIT, Munawar Kazmi. Submodules carry their own MIT licenses.
