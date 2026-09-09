// tests/test_shield.cpp
//
// Regression tests for the two assumptions the committed dataset kept
// hidden: that every map carries a lethal border wall, and that every
// grid is 0.05 m cells sitting at the world origin. Both hold for all
// forty qwen scenarios. Neither holds for a costmap captured off a
// running robot, so both are asserted here rather than left to the
// dataset to enforce by accident.
//
// The shield is a single translation unit whose helpers have internal
// linkage. Rather than split the pre-registered file, the test includes
// it with main() suppressed; every helper is exercised below, so the
// build stays clean under -Wall -Wextra -Wpedantic.
#define SHIELD_NO_MAIN
#include "../src/shield_main.cpp"

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                               \
  do {                                                 \
    if (!(cond)) {                                     \
      ++g_failures;                                    \
      std::printf("FAIL %s:%d  ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__);                        \
      std::printf("\n");                               \
    }                                                  \
  } while (0)

constexpr double kRes = 0.05;

// Cells of footprint-plus-margin inflation, as inflatedPlanningGrid
// computes it: 4 at the TurtleBot3 radius and 0.05 m cells.
std::int64_t inflationCells(double robot_radius, double res) {
  return static_cast<std::int64_t>(std::ceil((robot_radius + res) / res));
}

// A room with a two-cell vertical wall at x = wall_x blocking rows
// [wall_y0, wall_y1), so the gap can be placed above or below it.
// Border walls are optional: their absence is the whole point of the
// boundary tests.
verifier::Grid room(std::size_t w, std::size_t h, verifier::Point origin, bool walled,
                    std::size_t wall_x, std::size_t wall_y0, std::size_t wall_y1) {
  verifier::Grid g(w, h, kRes, origin, 0);
  if (walled) {
    for (std::size_t x = 0; x < w; ++x) {
      for (std::size_t b = 0; b < 2; ++b) {
        g.setCost(x, b, verifier::kLethal);
        g.setCost(x, h - 1 - b, verifier::kLethal);
      }
    }
    for (std::size_t y = 0; y < h; ++y) {
      for (std::size_t b = 0; b < 2; ++b) {
        g.setCost(b, y, verifier::kLethal);
        g.setCost(w - 1 - b, y, verifier::kLethal);
      }
    }
  }
  for (std::size_t y = wall_y0; y < wall_y1 && y < h; ++y) {
    g.setCost(wall_x, y, verifier::kLethal);
    g.setCost(wall_x + 1, y, verifier::kLethal);
  }
  return g;
}

verifier::Trajectory straight(verifier::Point a, verifier::Point b, double pitch = 0.2) {
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  const auto n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(len / pitch)));
  verifier::Trajectory t;
  for (std::size_t i = 0; i <= n; ++i) {
    const double s = static_cast<double>(i) / static_cast<double>(n);
    t.push_back({a.x + s * (b.x - a.x), a.y + s * (b.y - a.y)});
  }
  return t;
}

Record record(verifier::Point start, verifier::Point goal) {
  Record r{};
  r.scenario = 0;
  r.parse_ok = true;
  r.start = start;
  r.goal = goal;
  r.traj = straight(start, goal);  // drives through the wall: unsafe
  return r;
}

// ---------------------------------------------------------- boundary

// Out of bounds is inflated exactly as a lethal cell is, so a cell
// within infl of the edge is untraversable even on a map with no walls
// at all.
void testBorderBandIsInflated() {
  const verifier::Params params;
  const auto infl = inflationCells(params.robot_radius, kRes);
  CHECK(infl == 4, "expected 4 cells of inflation at the TurtleBot3 radius, got %lld",
        static_cast<long long>(infl));

  const verifier::Grid open = room(40, 40, {0.0, 0.0}, false, 0, 0, 0);
  const planning::Grid g = inflatedPlanningGrid(open, params.robot_radius);

  const auto band = static_cast<std::size_t>(infl);
  for (std::size_t y = 0; y < 40; ++y) {
    for (std::size_t x = 0; x < 40; ++x) {
      const bool in_band = x < band || y < band || x >= 40 - band || y >= 40 - band;
      const bool traversable = g.traversable(x, y);
      if (in_band) {
        CHECK(!traversable, "cell (%zu,%zu) is within %lld of the edge and must be lethal",
              x, y, static_cast<long long>(infl));
      } else {
        CHECK(traversable, "cell (%zu,%zu) is interior and must stay free", x, y);
      }
    }
  }
}

// The regression itself. On a map whose border cells are free, A* used
// to route along the edge and the recovery then failed re-verification
// with kOffMap, landing in the load-bearing cell. Whatever the shield
// decides now, it must never be fallback_unsafe.
void testFreeBorderNeverProducesUnsafeFallback() {
  const verifier::Params params;
  planning::AStarPlanner astar;

  struct Case {
    const char* name;
    std::size_t wall_y0, wall_y1;
    verifier::Point start, goal;
  };
  // The first three put an endpoint on the free edge. The fourth keeps
  // both endpoints well inside and leaves the only way past the wall
  // along the bottom edge, so the recovery hugs the boundary mid-route
  // rather than at either end. All four returned fallback_unsafe before
  // out of bounds was inflated.
  const Case cases[] = {
      {"start and goal hard against the free edge", 0, 40, {0.075, 0.075}, {2.875, 0.075}},
      {"start on the edge, goal inside", 0, 40, {0.075, 0.075}, {2.425, 0.575}},
      {"goal on the edge, start inside", 0, 40, {0.575, 0.575}, {2.875, 0.075}},
      {"endpoints inside, only route runs along the edge", 6, 60,
       {0.575, 1.275}, {2.425, 1.275}},
  };
  for (const auto& c : cases) {
    const verifier::Grid grid = room(60, 60, {0.0, 0.0}, false, 30, c.wall_y0, c.wall_y1);
    const Outcome o = runShield(grid, record(c.start, c.goal), params, astar);
    CHECK(o.bucket != "fallback_unsafe",
          "free border, %s: shield produced a fallback that failed re-verification", c.name);
  }
}

// The band must not swallow the map. With room to spare from every
// edge, an unsafe proposal still recovers, walled or not.
void testBorderBandStillLeavesRoomToRecover() {
  const verifier::Params params;
  planning::AStarPlanner astar;
  const Record r = record({0.575, 0.575}, {2.425, 0.575});

  for (const bool walled : {true, false}) {
    const verifier::Grid grid = room(60, 60, {0.0, 0.0}, walled, 30, 0, 40);
    const Outcome o = runShield(grid, r, params, astar);
    CHECK(o.bucket == "recovered_from_unsafe",
          "%s map with clearance: expected recovered_from_unsafe, got %s",
          walled ? "walled" : "free-border", o.bucket.c_str());
    CHECK(o.fallback_waypoints > 0, "a recovery must carry waypoints");
  }
}

// ------------------------------------------------------------- frames

void testSidecarSuppliesTheFrame() {
  const GridMeta fallback{0.05, {0.0, 0.0}};
  const char* path = "build/test_shield_meta.json";

  {
    std::ofstream f(path);
    f << R"({"id": 3, "width": 240, "resolution": 0.025,)"
      << R"( "origin": [-12.2, -8.4], "start": [1.0, 2.0]})";
  }
  const GridMeta both = loadGridMeta(path, fallback);
  CHECK(std::abs(both.resolution - 0.025) < 1e-12, "resolution from sidecar: got %g",
        both.resolution);
  CHECK(std::abs(both.origin.x + 12.2) < 1e-12 && std::abs(both.origin.y + 8.4) < 1e-12,
        "origin from sidecar: got (%g, %g)", both.origin.x, both.origin.y);

  // The committed scenarios state resolution and omit origin. A missing
  // field must keep the caller's value, never silently assert (0, 0).
  {
    std::ofstream f(path);
    f << R"({"id": 3, "resolution": 0.05, "ascii_cell_m": 0.25})";
  }
  const GridMeta partial = loadGridMeta(path, {0.05, {7.5, -3.25}});
  CHECK(std::abs(partial.origin.x - 7.5) < 1e-12 && std::abs(partial.origin.y + 3.25) < 1e-12,
        "absent origin must leave the fallback alone, got (%g, %g)", partial.origin.x,
        partial.origin.y);

  const GridMeta missing = loadGridMeta("build/test_shield_no_such_file.json",
                                        {0.1, {1.0, 2.0}});
  CHECK(std::abs(missing.resolution - 0.1) < 1e-12, "absent sidecar must keep the fallback");
  std::remove(path);
}

// A dumped grid carries its own frame, and a PGM with comment lines
// loads at all. The old reader took '#' as the width and failed on the
// first file any real producer wrote.
void testPgmCommentRoundTrip() {
  const char* path = "build/test_shield_grid.pgm";
  verifier::Grid original(12, 9, 0.025, {-12.2, -8.4}, 0);
  original.setCost(3, 4, verifier::kLethal);
  original.setCost(9, 2, verifier::kUnknown);
  writeCostPgm(path, original);

  // A deliberately wrong fallback: the comment must win.
  const auto loaded = loadCostPgm(path, {0.05, {99.0, 99.0}});
  CHECK(loaded.has_value(), "a grid written by writeCostPgm must load back");
  if (loaded) {
    CHECK(loaded->width() == 12 && loaded->height() == 9, "dimensions survive the round trip");
    CHECK(std::abs(loaded->resolution() - 0.025) < 1e-9, "resolution survives: got %g",
          loaded->resolution());
    CHECK(std::abs(loaded->origin().x + 12.2) < 1e-9 &&
              std::abs(loaded->origin().y + 8.4) < 1e-9,
          "origin survives: got (%g, %g)", loaded->origin().x, loaded->origin().y);
    CHECK(loaded->cost(3, 4) == verifier::kLethal && loaded->cost(9, 2) == verifier::kUnknown,
          "cell costs survive the round trip");
  }
  std::remove(path);

  // Comments in awkward places, and no metadata in them at all.
  {
    std::ofstream f(path, std::ios::binary);
    f << "P5\n# written by map_saver\n2 2\n# a comment before maxval\n255\n";
    const char cells[4] = {0, 0, 0, 0};
    f.write(cells, 4);
  }
  const auto plain = loadCostPgm(path, {0.05, {1.0, 2.0}});
  CHECK(plain.has_value(), "a PGM with comment lines must load");
  if (plain) {
    CHECK(plain->width() == 2 && plain->height() == 2, "comments must not be read as sizes");
    CHECK(std::abs(plain->origin().x - 1.0) < 1e-12,
          "a PGM with no metadata comment keeps the caller's frame");
  }
  std::remove(path);
}

// The same map and the same proposal, described in a different frame,
// must produce the same decision.
void testDecisionIsIndependentOfOrigin() {
  const verifier::Params params;
  planning::AStarPlanner astar;
  const verifier::Point offset{-12.2, -8.4};

  const verifier::Grid at_zero = room(60, 60, {0.0, 0.0}, true, 30, 0, 40);
  const Outcome a = runShield(at_zero, record({0.575, 0.575}, {2.425, 0.575}), params, astar);

  const verifier::Grid shifted = room(60, 60, offset, true, 30, 0, 40);
  const Outcome b = runShield(
      shifted, record({0.575 + offset.x, 0.575 + offset.y}, {2.425 + offset.x, 0.575 + offset.y}),
      params, astar);

  CHECK(a.bucket == b.bucket, "origin changed the decision: %s at zero, %s at (%g, %g)",
        a.bucket.c_str(), b.bucket.c_str(), offset.x, offset.y);
  CHECK(a.fallback_waypoints == b.fallback_waypoints,
        "origin changed the recovery length: %zu vs %zu", a.fallback_waypoints,
        b.fallback_waypoints);
  for (std::size_t i = 0; i < b.fallback.size() && i < a.fallback.size(); ++i) {
    CHECK(std::abs((b.fallback[i].x - offset.x) - a.fallback[i].x) < 1e-9 &&
              std::abs((b.fallback[i].y - offset.y) - a.fallback[i].y) < 1e-9,
          "recovery waypoint %zu does not match under the shift", i);
  }
}

// ------------------------------------------------- surrounding pieces

void testSealedGoalStillHalts() {
  const verifier::Params params;
  planning::AStarPlanner astar;
  CHECK(kHaltSuiteN == 10, "the pre-registered halt suite is ten scenarios");

  // Sealing must survive a nonzero origin too, since it locates the
  // goal through the grid's world transform.
  for (const verifier::Point origin : {verifier::Point{0.0, 0.0}, verifier::Point{-12.2, -8.4}}) {
    const verifier::Grid grid = room(60, 60, origin, true, 30, 0, 40);
    const Record r = record({0.575 + origin.x, 0.575 + origin.y},
                            {2.425 + origin.x, 0.575 + origin.y});
    const verifier::Grid sealed = sealGoal(grid, r.goal);
    const Outcome o = runShield(sealed, r, params, astar);
    CHECK(o.bucket == "halted_no_safe_path",
          "sealed goal at origin (%g, %g): expected a halt, got %s", origin.x, origin.y,
          o.bucket.c_str());
  }
}

void testDatasetLoaders() {
  const char* parsed_path = "build/test_shield_parsed.txt";
  {
    std::ofstream f(parsed_path);
    f << "scenario 7\nparse_ok 1\nstart 1.5 2.5\ngoal 3.5 4.5\nwaypoints 2\n"
      << "1.5 2.5\n3.5 4.5\n";
  }
  const auto records = loadParsed(parsed_path);
  CHECK(records.size() == 1, "expected one record, got %zu", records.size());
  if (!records.empty()) {
    const Record& r = records.front();
    CHECK(r.scenario == 7 && r.parse_ok, "scenario id and parse flag");
    CHECK(std::abs(r.start.x - 1.5) < 1e-12 && std::abs(r.goal.y - 4.5) < 1e-12,
          "start and goal round-trip");
    CHECK(r.traj.size() == 2, "waypoint count, got %zu", r.traj.size());
  }
  std::remove(parsed_path);

  const char* traj_path = "build/test_shield_traj.txt";
  const verifier::Grid g(10, 10, kRes, {-1.0, -2.0}, 0);
  const planning::Path cells{{2, 3}, {3, 4}};
  const verifier::Trajectory t = cellsToTrajectory(g, cells);
  CHECK(t.size() == 2, "cellsToTrajectory keeps one point per cell");
  if (t.size() == 2) {
    CHECK(std::abs(t[0].x - (-1.0 + 2.5 * kRes)) < 1e-12,
          "cell centers are taken through the grid's own transform, got %g", t[0].x);
  }
  writeTrajectory(traj_path, t);
  std::ifstream check(traj_path);
  CHECK(check.good(), "writeTrajectory produced a readable file");
  std::remove(traj_path);
}

}  // namespace

int main() {
  testBorderBandIsInflated();
  testFreeBorderNeverProducesUnsafeFallback();
  testBorderBandStillLeavesRoomToRecover();
  testSidecarSuppliesTheFrame();
  testPgmCommentRoundTrip();
  testDecisionIsIndependentOfOrigin();
  testSealedGoalStillHalts();
  testDatasetLoaders();

  if (g_failures == 0) {
    std::printf("PASS: boundary and frame handling hold on maps the committed "
                "dataset cannot produce\n");
    return 0;
  }
  std::printf("FAIL: %d check%s failed\n", g_failures, g_failures == 1 ? "" : "s");
  return 1;
}
