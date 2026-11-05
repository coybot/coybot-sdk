// Copyright 2026 Coybot AI, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Replays a saved planning problem (planner_node's dump_dir) offline and says where it fails:
//   replay_plan stuck_1.bin [key=value ...]   (keys: corridor_margin_m, rrt_timeout_s, horizon_m, max_vel,
//                                              inflate_m is fixed by the saved map)
// For each attempt that meets the map: is the bad sample inside a FIRI polytope (the corridor itself contains a
// blocked voxel) or outside every one (the trajectory left the corridor)? Units: m, s.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

#include "coybot_avoidance/local_planner.hpp"

using namespace coybot_avoidance;

static double depth_in(const Eigen::MatrixX4d &h, const Eigen::Vector3d &p)
{
  // max over faces of the signed distance (normalised): <= 0 inside
  const Eigen::ArrayXd n = h.leftCols<3>().rowwise().norm();
  return ((h.leftCols<3>() * p + h.col(3)).array() / n).maxCoeff();
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: replay_plan snapshot.bin [key=value ...]\n");
    return 2;
  }
  Params P;
  int repeats = 5;
  bool stuck = true;
  for (int i = 2; i < argc; i++)
  {
    const char *eq = std::strchr(argv[i], '=');
    if (!eq)
      continue;
    const std::string k(argv[i], eq - argv[i]);
    const double v = std::atof(eq + 1);
    if (k == "corridor_margin_m") P.corridor_margin_m = v;
    else if (k == "rrt_timeout_s") P.rrt_timeout_s = v;
    else if (k == "rrt_retry_timeout_s") P.rrt_retry_timeout_s = v;
    else if (k == "partial_route_goal") P.partial_route_goal = v != 0;
    else if (k == "stuck") stuck = v != 0;
    else if (k == "escape_timeout_s") P.escape_timeout_s = v;
    else if (k == "horizon_m") P.horizon_m = v;
    else if (k == "max_vel") P.max_vel = v;
    else if (k == "repeats") repeats = int(v);
    else std::fprintf(stderr, "unknown key %s\n", k.c_str());
  }
  LocalPlanner pl(P);
  std::ifstream is(argv[1], std::ios::binary);
  double now;
  State odom;
  if (!pl.load(is, now, odom))
  {
    std::fprintf(stderr, "cannot read %s\n", argv[1]);
    return 1;
  }
  std::printf("goal [%.2f %.2f %.2f], odom [%.2f %.2f %.2f] at %.2f m/s\n", pl.goal()(0), pl.goal()(1), pl.goal()(2),
              odom.p(0), odom.p(1), odom.p(2), odom.v.norm());
  int passed = 0;
  for (int r = 0; r < repeats; r++)  // OMPL's route search is random: replay a few times
  {
    Reference out;
    std::string why;
    PlanDebug dbg;
    const bool ok = pl.replan(now, out, why, dbg, stuck);
    passed += ok;
    double rl = 0;
    for (size_t i = 1; i < dbg.route.size(); i++)
      rl += (dbg.route[i] - dbg.route[i - 1]).norm();
    std::printf("\n#%d %s%s\n  start [%.2f %.2f %.2f] (map %d), local goal [%.2f %.2f %.2f] (map %d)\n", r,
                ok ? "PLAN" : "no plan: ", ok ? "" : why.c_str(), dbg.start(0), dbg.start(1), dbg.start(2),
                pl.map().query(dbg.start), dbg.local_goal(0), dbg.local_goal(1), dbg.local_goal(2),
                pl.map().query(dbg.local_goal));
    if (!dbg.escape.empty())
      std::printf("  escaping: route to the goal %zu points\n", dbg.escape.size());
    std::printf("  route %zu points, %.1f m; corridor %zu polytopes; start depth in first %.3f m, goal in last %.3f m\n",
                dbg.route.size(), rl, dbg.corridor.size(),
                dbg.corridor.empty() ? NAN : depth_in(dbg.corridor.front(), dbg.start),
                dbg.corridor.empty() ? NAN : depth_in(dbg.corridor.back(), dbg.local_goal));
    for (const auto &at : dbg.attempts)
    {
      std::printf("  margin %.2f: %s", at.margin, at.why.empty() ? "passed" : at.why.c_str());
      if (at.bad_sample >= 0 && at.ref.ok())
      {
        const Eigen::Vector3d q = at.ref.p[at.bad_sample];
        double best = INFINITY;
        int k = -1;
        for (size_t j = 0; j < dbg.corridor.size(); j++)
        {
          const double d = depth_in(dbg.corridor[j], q);
          if (d < best)
            best = d, k = int(j);
        }
        std::printf("; bad sample t %.2f s of %.2f s at [%.2f %.2f %.2f], speed %.2f m/s; %s polytope %d by %.3f m",
                    at.bad_sample * at.ref.dt, at.ref.duration(), q(0), q(1), q(2), at.ref.v[at.bad_sample].norm(),
                    best <= 0 ? "INSIDE" : "outside nearest", k, std::abs(best));
      }
      std::printf("\n");
    }
  }
  std::printf("\n%d of %d replays found a plan\n", passed, repeats);
  return 0;
}
