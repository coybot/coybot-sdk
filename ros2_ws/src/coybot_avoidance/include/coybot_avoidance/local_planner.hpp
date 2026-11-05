// Copyright 2026 Coybot AI, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Map-free local planner for multirotors: plans from a depth camera alone, no prior map.
//
// Each step():
//   start     = the current reference at now + lead_s (stitched), or the odometry for the first plan;
//   local goal = horizon_m along the straight line to the goal, moved to the nearest point with goal_margin_m of
//               free space around it if needed; when that fails after escape_after_s without a plan, horizon_m
//               along a route to the goal searched across the whole map;
//   route     = OMPL InformedRRT* in a box around start and local goal (GCOPTER's sfc_gen::planPath). OMPL
//               reports a partial route as solved; a route that stops short of the local goal is searched
//               again for longer, and then the local goal moves to its end (if roomy) or there is no plan;
//   corridor  = FIRI convex polytopes along the route (sfc_gen::convexCover + shortCut), each face moved
//               inward by corridor_margin_m (FIRI keeps voxel centres out; the check rejects whole voxels);
//   trajectory = GCOPTER_PolytopeSFC, ending at rest at the local goal, within speed, tilt, body-rate and thrust;
//   check     = sampled every check_dt_s against the grown map, rejected if it meets it, is not finite, is shorter
//               than min_plan_s or exceeds the speed cap by 10 %; on failure retried once with the unshrunk
//               corridor.
// No valid plan: keep the current reference while it is clear; if it meets the map nearer than the stopping
// distance v^2 / (2 brake_acc) + v brake_latency_s + brake_margin_m, replace it with a straight stop at brake_acc.
//
// Map: voxels of map_res_m; a voxel is occupied after hit_count depth hits and grows a sphere of inflate_m as it
// becomes occupied. Nothing is ever cleared (static scenes). The map covers a box around start and goal, fixed
// when the goal is set; outside it counts as blocked.
//
// Units: m, s, rad, kg, N; positions in one world frame (ENU in the ROS node), z up.
#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace coybot_avoidance
{

struct Params
{
  double max_vel = 4.0;           // m/s, speed cap
  double horizon_m = 12.0;        // local goal distance
  double lead_s = 0.15;           // a new plan starts this far ahead on the current reference
  double map_res_m = 0.1;
  double inflate_m = 0.8;         // keep-out from any seen surface (vehicle radius included)
  int hit_count = 2;              // depth hits before a voxel counts as occupied
  double max_range_m = 15.0;      // depth beyond this is ignored
  int skip_pixel = 2;             // use every n-th depth pixel in each direction
  double slab_m = 1.0;            // plans stay within +- slab_m of the goal altitude
  double rrt_timeout_s = 0.02;
  double rrt_retry_timeout_s = 0.1;  // second route search when the first does not reach the local goal
  bool partial_route_goal = true;    // if neither does, aim for the end of the partial route (if roomy)
  // Escape from dead ends: after escape_after_s without a new plan, when the usual plan fails, try once more with a
  // local goal along a route to the goal itself searched across the whole map (escape_timeout_s); that route is
  // reused for escape_hold_s while it stays clear.
  double escape_after_s = 1.0;
  double escape_timeout_s = 0.3;
  double escape_hold_s = 8.0;
  double check_dt_s = 0.02;
  double min_plan_s = 0.3;        // shorter plans are rejected (degenerate near the goal)
  double goal_margin_m = 0.3;     // free space required around a local goal
  double corridor_margin_m = 0.15; // corridor faces moved inward by this before GCOPTER (0 = off); retried
                                  // without it if that fails
  double final_approach_m = 1.5;  // no replanning this close to the goal when the reference already ends there
  double arrive_m = 0.3;
  double brake_acc = 3.0;         // m/s^2
  double brake_latency_s = 0.3;
  double brake_margin_m = 0.5;
  // GCOPTER vehicle model and weights (x500-class, 2.0 kg, no drag)
  double max_tilt_rad = 0.40;
  double max_bodyrate = 2.1;      // rad/s
  double mass_kg = 2.0;
  double thrust_min_n = 5.0;
  double thrust_max_n = 40.0;
  double weight_t = 20.0;
  std::vector<double> chi{1e4, 1e4, 1e4, 1e4, 1e5};
  double smoothing_eps = 1e-2;
  int integral_intervs = 16;
  double rel_cost_tol = 1e-5;
  double map_margin_xy_m = 15.0;  // map extent beyond the start-goal box
  double map_top_m = 2.5;         // map extends from z = 0 to goal altitude + this
};

// Occupancy grid with growth: 0 free or unknown, 1 occupied, 2 within inflate_m of an occupied voxel.
class VoxelMap
{
public:
  bool ready() const { return !v_.empty(); }
  void init(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi, double res, double inflate, int hit_count);
  void hit(const Eigen::Vector3d &p);
  // 0 = free (sfc_gen::planPath's convention); outside the map counts as blocked.
  int query(const Eigen::Vector3d &p) const;
  // Boundary voxels of the grown region inside a box (the points FIRI keeps corridors away from), every other z.
  void surface(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi, std::vector<Eigen::Vector3d> &pts) const;
  const Eigen::Vector3d &origin() const { return o_; }
  // Binary snapshot of the grid (for offline replay of a planning problem).
  void save(std::ostream &os) const;
  bool load(std::istream &is);
  Eigen::Vector3d extent() const { return n_.cast<double>() * res_; }
  double resolution() const { return res_; }

private:
  bool in(const Eigen::Vector3i &id) const { return (id.array() >= 0).all() && (id.array() < n_.array()).all(); }
  size_t at(const Eigen::Vector3i &id) const
  {
    return size_t(id(0)) + size_t(n_(0)) * (size_t(id(1)) + size_t(n_(1)) * size_t(id(2)));
  }
  Eigen::Vector3i idx(const Eigen::Vector3d &p) const { return ((p - o_) / res_).array().floor().cast<int>(); }
  Eigen::Vector3d centre(const Eigen::Vector3i &id) const
  {
    return o_ + (id.cast<double>().array() + 0.5).matrix() * res_;
  }
  Eigen::Vector3d o_ = Eigen::Vector3d::Zero();
  double res_ = 0.1;
  Eigen::Vector3i n_ = Eigen::Vector3i::Zero();
  std::vector<uint8_t> v_, hits_;
  std::vector<Eigen::Vector3i> ball_;
  int hit_count_ = 2;
};

// A reference sampled every dt from t0: position, velocity, acceleration, clamped at the ends (at rest after).
struct Reference
{
  double t0 = 0.0, dt = 0.01;
  std::vector<Eigen::Vector3d> p, v, a;
  bool ok() const { return !p.empty(); }
  double duration() const { return p.size() > 1 ? dt * double(p.size() - 1) : 0.0; }
  void eval(double t, Eigen::Vector3d &pp, Eigen::Vector3d &vv, Eigen::Vector3d &aa) const;
};

// One depth image in the planner's world frame. Depth is 32FC1 in m, or 16UC1 in mm.
struct DepthView
{
  const uint8_t *data = nullptr;
  int width = 0, height = 0;
  size_t step = 0;                // bytes per row
  bool float32 = true;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  Eigen::Matrix3d R_world_cam = Eigen::Matrix3d::Identity();   // camera optical frame (z forward) to world
  Eigen::Vector3d t_world_cam = Eigen::Vector3d::Zero();
};

struct State
{
  Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
};

enum class Outcome
{
  Idle,       // no goal, or no map yet
  Plan,       // a new reference: publish it
  Keep,       // no new plan, the current reference is still clear (or this is the final approach)
  Brake,      // a straight stop replaced the reference: publish it
  Stuck,      // no new plan, nothing to keep, already braking or at rest
  Arrived,    // at the goal; nothing more until a new goal
};
const char *to_string(Outcome o);

// What one planning attempt produced, for offline diagnosis (filled only when LocalPlanner::debug is set).
struct PlanDebug
{
  Eigen::Vector3d start = Eigen::Vector3d::Zero(), local_goal = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> route, escape;    // escape: the route to the goal, when escaping
  std::vector<Eigen::MatrixX4d> corridor;          // as FIRI built it (before any margin)
  struct Attempt
  {
    double margin = 0.0;
    std::string why;                               // empty when it passed
    Reference ref;
    int bad_sample = -1;                           // index into ref.p where it met the map, if it did
  };
  std::vector<Attempt> attempts;
};

struct StepResult
{
  Outcome outcome = Outcome::Idle;
  std::string detail;             // why no plan, when there is none
};

class LocalPlanner
{
public:
  explicit LocalPlanner(const Params &p);
  // Sets the goal; the first goal also sizes the map around start and goal.
  void setGoal(const Eigen::Vector3d &goal, const Eigen::Vector3d &start);
  bool hasGoal() const { return have_goal_; }
  bool arrived() const { return done_; }
  const Eigen::Vector3d &goal() const { return goal_; }
  // Adds one depth image (pixels outside 0.2 m .. max_range_m, or outside the altitude slab, are ignored).
  void insertDepth(const DepthView &d);
  // Adds one world point (for other sensors and tests).
  void insertPoint(const Eigen::Vector3d &p) { map_.hit(p); }
  // One planning cycle at time now (the clock the reference is timed in). odom is the latest estimate.
  StepResult step(double now, const State &odom);
  // The reference to follow (valid after a Plan or Brake outcome).
  const Reference &reference() const { return cur_; }
  const VoxelMap &map() const { return map_; }
  // Snapshot of the planning problem at time now (map, goal, reference, odometry) and its replay: plan once from
  // the snapshot's start state without changing anything, filling dbg.
  void save(std::ostream &os, double now, const State &odom) const;
  bool load(std::istream &is, double &now, State &odom);
  // stuck: as if no plan had passed for longer than escape_after_s (so the escape search runs).
  bool replan(double now, Reference &out, std::string &why, PlanDebug &dbg, bool stuck);
  const Params &params() const { return P_; }

private:
  bool roomy(const Eigen::Vector3d &q) const;
  bool plan(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &a, double ts, Reference &out,
            std::string &why, bool escape);
  bool optimise(const std::vector<Eigen::MatrixX4d> &hpolys, const Eigen::Matrix3d &ini, const Eigen::Matrix3d &fin,
                double ts, Reference &out, std::string &why) const;
  double blockedAhead(const Reference &r, double t) const;
  bool record(bool ok, double margin, const Reference &r, const std::string &why) const;

  Params P_;
  PlanDebug *dbg_ = nullptr;
  double last_ok_t_ = -INFINITY;               // last time a plan passed (or the first step)
  std::vector<Eigen::Vector3d> escape_;         // route to the goal, while escaping
  double escape_t_ = -INFINITY;
  bool escapeGoal(const Eigen::Vector3d &p, const Eigen::Vector3d &ps, double ts, Eigen::Vector3d &lg);
  VoxelMap map_;
  Reference cur_;
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  bool have_goal_ = false, done_ = false, braking_ = false;
};

}  // namespace coybot_avoidance
