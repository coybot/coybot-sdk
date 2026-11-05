// Copyright 2026 Coybot AI, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LocalPlanner without ROS: a synthetic forest of vertical cylinders, a simple depth sensor (points on the facing
// half of each trunk within range and field of view, no occlusion), and perfect tracking (the vehicle is where the
// reference says). Units: m, s.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "coybot_avoidance/local_planner.hpp"

using coybot_avoidance::LocalPlanner;
using coybot_avoidance::Outcome;
using coybot_avoidance::Params;
using coybot_avoidance::Reference;
using coybot_avoidance::State;
using Eigen::Vector3d;

namespace
{
struct Trunk
{
  double x, y, r;
};

// Points on the half of each trunk facing the sensor, every 5 cm around and 10 cm up, within range and +-45 deg of
// the heading, inserted twice (hit_count 2).
void sense(LocalPlanner &pl, const std::vector<Trunk> &trunks, const Vector3d &p, double heading, double range)
{
  for (const auto &t : trunks)
  {
    const Vector3d c(t.x, t.y, p(2));
    const Vector3d d = c - p;
    const double dist = std::hypot(d(0), d(1)) - t.r;
    if (dist > range)
      continue;
    double off = std::atan2(d(1), d(0)) - heading;
    off = std::atan2(std::sin(off), std::cos(off));
    if (std::abs(off) > M_PI / 4 + std::atan2(t.r, std::max(dist, 0.1)))
      continue;
    const double face = std::atan2(-d(1), -d(0));
    const int n = std::max(8, int(M_PI * t.r / 0.05));
    for (int i = 0; i <= n; i++)
    {
      const double a = face - M_PI / 2 + M_PI * i / n;
      for (double z = p(2) - 1.4; z <= p(2) + 1.4; z += 0.1)
      {
        const Vector3d q(t.x + t.r * std::cos(a), t.y + t.r * std::sin(a), z);
        pl.insertPoint(q);
        pl.insertPoint(q);
      }
    }
  }
}

double clearance(const std::vector<Trunk> &trunks, const Vector3d &p)
{
  double m = INFINITY;
  for (const auto &t : trunks)
    m = std::min(m, std::hypot(p(0) - t.x, p(1) - t.y) - t.r);
  return m;
}

// Every reference handed out must be finite and not degenerate (round G1's overshoot came from a 0.1 s plan with
// non-finite or all-zero samples near the goal).
void expectSane(const Reference &r, const Params &P, Outcome o)
{
  ASSERT_TRUE(r.ok());
  if (o == Outcome::Plan)
    EXPECT_GE(r.duration(), P.min_plan_s - 1e-9);
  for (size_t i = 0; i < r.p.size(); i++)
  {
    ASSERT_TRUE(r.p[i].allFinite() && r.v[i].allFinite() && r.a[i].allFinite()) << "sample " << i;
    EXPECT_LE(r.v[i].norm(), 1.1 * P.max_vel + 1e-6);
  }
}

struct Flight
{
  bool arrived = false;
  double min_clearance = INFINITY, t_end = 0;
  int plans = 0, brakes = 0;
  Vector3d end = Vector3d::Zero();
};

Flight fly(LocalPlanner &pl, const std::vector<Trunk> &trunks, const Vector3d &start, const Vector3d &goal,
           double t_max)
{
  Flight f;
  pl.setGoal(goal, start);
  State s;
  s.p = start;
  bool have_ref = false;
  for (double now = 0.0; now < t_max; now += 0.1)
  {
    if (have_ref)
    {
      Vector3d a;
      pl.reference().eval(now, s.p, s.v, a);
    }
    const double heading = s.v.head<2>().norm() > 0.3 ? std::atan2(s.v(1), s.v(0))
                                                     : std::atan2(goal(1) - s.p(1), goal(0) - s.p(0));
    sense(pl, trunks, s.p, heading, pl.params().max_range_m);
    f.min_clearance = std::min(f.min_clearance, clearance(trunks, s.p));
    const auto r = pl.step(now, s);
    if (r.outcome == Outcome::Plan || r.outcome == Outcome::Brake)
    {
      expectSane(pl.reference(), pl.params(), r.outcome);
      have_ref = true;
      (r.outcome == Outcome::Plan ? f.plans : f.brakes)++;
    }
    if (r.outcome == Outcome::Arrived)
    {
      f.arrived = true;
      f.t_end = now;
      break;
    }
  }
  f.end = s.p;
  // the whole followed path, finely, for clearance
  return f;
}

std::vector<Trunk> forest(unsigned seed, double spacing)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jit(-0.35 * spacing, 0.35 * spacing), rad(0.15, 0.3);
  std::vector<Trunk> t;
  for (double x = 6.0; x < 62.0; x += spacing)
    for (double y = -12.0; y <= 12.0; y += spacing)
    {
      Trunk k{x + jit(rng), y + jit(rng), rad(rng)};
      if (std::hypot(k.x, k.y) > 3.0 && std::hypot(k.x - 70.0, k.y) > 3.0)
        t.push_back(k);
    }
  return t;
}
}  // namespace

TEST(LocalPlanner, OpenSpaceArrivesAndStops)
{
  Params P;
  LocalPlanner pl(P);
  const auto f = fly(pl, {}, Vector3d(0, 0, 2.5), Vector3d(30, 0, 2.5), 40.0);
  ASSERT_TRUE(f.arrived);
  EXPECT_LT((f.end - Vector3d(30, 0, 2.5)).norm(), P.arrive_m + 0.05);
  EXPECT_EQ(f.brakes, 0);
  // after arriving, nothing more happens
  State s;
  s.p = f.end;
  EXPECT_EQ(pl.step(f.t_end + 1.0, s).outcome, Outcome::Arrived);
}

TEST(LocalPlanner, ForestReachesGoalWithClearance)
{
  for (double spacing : {5.0, 3.5})
  for (unsigned seed : {1u, 2u, 3u})
    for (double v : {4.0, 8.0})
    {
      Params P;
      P.max_vel = v;
      LocalPlanner pl(P);
      const auto trunks = forest(seed, spacing);
      const auto f = fly(pl, trunks, Vector3d(0, 0, 2.5), Vector3d(70, 0, 2.5), 90.0);
      EXPECT_TRUE(f.arrived) << "spacing " << spacing << " m, seed " << seed << " at " << v << " m/s, ended at " << f.end.transpose();
      // keep-out 0.8 m in 0.1 m voxels: the centre can come to within a voxel diagonal of it
      EXPECT_GT(f.min_clearance, P.inflate_m - 0.2) << "seed " << seed << " at " << v << " m/s";
    }
}

TEST(LocalPlanner, LocalGoalHasRoomAroundIt)
{
  // A trunk whose grown region ends 0.1 m past the 12 m point on the line: that point is free but cramped.
  Params P;
  LocalPlanner pl(P);
  const std::vector<Trunk> trunks{{12.0 + 0.1 + P.inflate_m + 1.0, 0.0, 1.0}};
  const Vector3d start(0, 0, 2.5), goal(40, 0, 2.5);
  pl.setGoal(goal, start);
  State s;
  s.p = start;
  sense(pl, trunks, start, 0.0, P.max_range_m);
  const auto r = pl.step(0.0, s);
  ASSERT_EQ(r.outcome, Outcome::Plan) << r.detail;
  const Vector3d end = pl.reference().p.back();
  for (int k = 0; k < 8; k++)
  {
    const double th = k * M_PI / 4;
    EXPECT_EQ(pl.map().query(end + P.goal_margin_m * Vector3d(std::cos(th), std::sin(th), 0)), 0) << "dir " << k;
  }
}

TEST(LocalPlanner, StopsWhenTheWayIsShut)
{
  // Flying at 4 m/s in the open, a wall wider than the local-goal search appears 3 m ahead of its keep-out edge.
  // Either a planned stop or the straight brake is right; the vehicle must stop outside the keep-out.
  Params P;
  LocalPlanner pl(P);
  const Vector3d start(0, 0, 2.5), goal(60, 0, 2.5);
  pl.setGoal(goal, start);
  State s;
  s.p = start;
  double now = 0.0;
  for (; now < 20.0; now += 0.1)
  {
    if (pl.reference().ok())
    {
      Vector3d a;
      pl.reference().eval(now, s.p, s.v, a);
    }
    if (s.v.norm() > 3.5)
      break;
    pl.step(now, s);
  }
  ASSERT_GT(s.v.norm(), 3.5);
  const double wall_x = s.p(0) + 3.0 + P.inflate_m;
  for (double y = -20; y <= 20; y += 0.05)
    for (double z = 1.0; z <= 4.5; z += 0.1)
    {
      pl.insertPoint(Vector3d(wall_x, y, z));
      pl.insertPoint(Vector3d(wall_x, y, z));
    }
  double x_max = s.p(0);
  int brakes = 0;
  for (double t_end = now + 6.0; now < t_end; now += 0.1)
  {
    Vector3d a;
    pl.reference().eval(now, s.p, s.v, a);
    x_max = std::max(x_max, s.p(0));
    const auto r = pl.step(now, s);
    if (r.outcome == Outcome::Plan || r.outcome == Outcome::Brake)
      expectSane(pl.reference(), P, r.outcome);
    if (r.outcome == Outcome::Brake)
    {
      brakes++;
      const Reference &b = pl.reference();
      for (size_t i = 1; i < b.v.size(); i++)
        EXPECT_LE((b.v[i] - b.v[i - 1]).norm() / b.dt, P.brake_acc + 1e-6);
    }
  }
  EXPECT_LT(s.v.norm(), 0.05);
  EXPECT_LT(x_max, wall_x - P.inflate_m + P.map_res_m) << brakes << " brakes";
}
