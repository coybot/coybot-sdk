// Copyright 2026 Coybot AI, Inc.
// SPDX-License-Identifier: Apache-2.0
#include "coybot_avoidance/local_planner.hpp"

#include <algorithm>
#include <cmath>

// sfc_gen.hpp (with geo_utils.hpp) first: gcopter.hpp uses geo_utils without including it.
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"

namespace coybot_avoidance
{

const char *to_string(Outcome o)
{
  switch (o)
  {
  case Outcome::Idle: return "idle";
  case Outcome::Plan: return "plan";
  case Outcome::Keep: return "keep";
  case Outcome::Brake: return "brake";
  case Outcome::Stuck: return "stuck";
  case Outcome::Arrived: return "arrived";
  }
  return "?";
}

// ---------------------------------------------------------------- VoxelMap

void VoxelMap::init(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi, double res, double inflate, int hit_count)
{
  res_ = res;
  o_ = lo;
  n_ = ((hi - lo) / res).cast<int>();
  v_.assign(size_t(n_.prod()), 0);
  hits_.assign(v_.size(), 0);
  hit_count_ = hit_count;
  const int k = int(std::ceil(inflate / res));
  ball_.clear();
  for (int i = -k; i <= k; i++)
    for (int j = -k; j <= k; j++)
      for (int l = -k; l <= k; l++)
        if (Eigen::Vector3d(i, j, l).norm() * res <= inflate + 1e-9)
          ball_.emplace_back(i, j, l);
}

void VoxelMap::hit(const Eigen::Vector3d &p)
{
  if (!ready() || !p.allFinite())
    return;
  const Eigen::Vector3i id = idx(p);
  if (!in(id))
    return;
  const size_t a = at(id);
  if (v_[a] == 1 || ++hits_[a] < hit_count_)
    return;
  v_[a] = 1;
  for (const auto &d : ball_)
  {
    const Eigen::Vector3i q = id + d;
    if (in(q) && v_[at(q)] == 0)
      v_[at(q)] = 2;
  }
}

int VoxelMap::query(const Eigen::Vector3d &p) const
{
  if (!p.allFinite())
    return 1;
  const Eigen::Vector3i id = idx(p);
  return in(id) ? int(v_[at(id)]) : 1;
}

void VoxelMap::surface(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi, std::vector<Eigen::Vector3d> &pts) const
{
  const Eigen::Vector3i a = idx(lo).cwiseMax(Eigen::Vector3i::Zero());
  const Eigen::Vector3i b = idx(hi).cwiseMin(n_ - Eigen::Vector3i::Ones());
  static const int nb[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (int z = a(2); z <= b(2); z += 2)
    for (int y = a(1); y <= b(1); y++)
      for (int x = a(0); x <= b(0); x++)
      {
        const Eigen::Vector3i id(x, y, z);
        if (v_[at(id)] == 0)
          continue;
        for (const auto &d : nb)
        {
          const Eigen::Vector3i q = id + Eigen::Vector3i(d[0], d[1], d[2]);
          if (in(q) && v_[at(q)] == 0)
          {
            pts.push_back(centre(id));
            break;
          }
        }
      }
}

// ---------------------------------------------------------------- Reference

void Reference::eval(double t, Eigen::Vector3d &pp, Eigen::Vector3d &vv, Eigen::Vector3d &aa) const
{
  if (p.size() == 1)
  {
    pp = p[0], vv.setZero(), aa.setZero();
    return;
  }
  const double s = std::clamp((t - t0) / dt, 0.0, double(p.size() - 1));
  const size_t i = std::min(size_t(s), p.size() - 2);
  const double f = s - double(i);
  pp = (1 - f) * p[i] + f * p[i + 1];
  vv = (1 - f) * v[i] + f * v[i + 1];
  aa = (1 - f) * a[i] + f * a[i + 1];
  if (t - t0 >= duration())
    vv.setZero(), aa.setZero();
}

// ---------------------------------------------------------------- LocalPlanner

LocalPlanner::LocalPlanner(const Params &p) : P_(p) {}

void LocalPlanner::setGoal(const Eigen::Vector3d &goal, const Eigen::Vector3d &start)
{
  goal_ = goal;
  if (!map_.ready())
  {
    const Eigen::Vector3d m(P_.map_margin_xy_m, P_.map_margin_xy_m, 0.0);
    Eigen::Vector3d lo = start.cwiseMin(goal) - m, hi = start.cwiseMax(goal) + m;
    lo(2) = 0.0;
    hi(2) = goal(2) + P_.map_top_m;
    map_.init(lo, hi, P_.map_res_m, P_.inflate_m, P_.hit_count);
  }
  have_goal_ = true;
  done_ = false;
}

void LocalPlanner::insertDepth(const DepthView &d)
{
  if (!map_.ready() || !have_goal_ || d.data == nullptr)
    return;
  const double z_lo = goal_(2) - P_.slab_m - 0.5, z_hi = goal_(2) + P_.slab_m + 0.5;
  const int k = std::max(1, P_.skip_pixel);
  for (int r = 0; r < d.height; r += k)
    for (int c = 0; c < d.width; c += k)
    {
      const uint8_t *row = d.data + size_t(r) * d.step;
      const double z = d.float32 ? double(reinterpret_cast<const float *>(row)[c])
                                 : 1e-3 * double(reinterpret_cast<const uint16_t *>(row)[c]);
      if (!std::isfinite(z) || z < 0.2 || z >= P_.max_range_m)
        continue;
      const Eigen::Vector3d pw = d.R_world_cam * Eigen::Vector3d((c - d.cx) * z / d.fx, (r - d.cy) * z / d.fy, z) +
                                 d.t_world_cam;
      if (pw(2) >= z_lo && pw(2) <= z_hi)
        map_.hit(pw);
    }
}

double LocalPlanner::blockedAhead(const Reference &r, double t) const
{
  Eigen::Vector3d p, v, a, prev;
  r.eval(t, prev, v, a);
  double s = 0.0;
  for (double tt = t; tt <= r.t0 + r.duration(); tt += P_.check_dt_s)
  {
    r.eval(tt, p, v, a);
    s += (p - prev).norm();
    prev = p;
    if (map_.query(p) != 0)
      return s;
  }
  return INFINITY;
}

StepResult LocalPlanner::step(double now, const State &odom)
{
  StepResult res;
  if (!have_goal_ || !map_.ready() || done_)
  {
    res.outcome = done_ ? Outcome::Arrived : Outcome::Idle;
    return res;
  }
  const double ts = now + P_.lead_s;
  Eigen::Vector3d p, v, a;
  if (cur_.ok())
    cur_.eval(ts, p, v, a);
  else
    p = odom.p, v = odom.v, a.setZero();
  if ((p - goal_).norm() < P_.arrive_m && cur_.ok() && ts > cur_.t0 + cur_.duration())
  {
    done_ = true;
    res.outcome = Outcome::Arrived;
    return res;
  }
  // Final approach: the reference already ends at rest at the goal. A GCOPTER trajectory of a few tens of cm can
  // come back degenerate (0.1 s long, non-finite); keep the one we have.
  if (cur_.ok() && (p - goal_).norm() < P_.final_approach_m && (cur_.p.back() - goal_).norm() < P_.arrive_m)
  {
    res.outcome = Outcome::Keep;
    res.detail = "final approach";
    return res;
  }
  Reference r;
  if (plan(p, v, a, ts, r, res.detail))
  {
    cur_ = r;
    braking_ = false;
    res.outcome = Outcome::Plan;
    return res;
  }
  if (!cur_.ok())
  {
    res.outcome = Outcome::Stuck;
    return res;
  }
  // No new plan: keep the reference while it is clear; stop when it is blocked within stopping distance.
  const double sp = odom.v.norm();
  const double d_stop = sp * sp / (2 * P_.brake_acc) + sp * P_.brake_latency_s + P_.brake_margin_m;
  const double s = blockedAhead(cur_, now);
  if (s < d_stop && !braking_)
  {
    Reference b;
    b.t0 = ts;
    b.dt = 0.01;
    const double vn = v.norm();
    const Eigen::Vector3d dir = vn > 1e-6 ? Eigen::Vector3d(v / vn) : Eigen::Vector3d::Zero();
    const double T = vn / P_.brake_acc + 0.5;
    for (double t = 0; t <= T + 1e-9; t += b.dt)
    {
      const double tb = std::min(t, vn / P_.brake_acc);
      b.p.push_back(p + dir * (vn * tb - 0.5 * P_.brake_acc * tb * tb));
      b.v.push_back(dir * std::max(0.0, vn - P_.brake_acc * t));
      b.a.push_back(t < vn / P_.brake_acc ? Eigen::Vector3d(-P_.brake_acc * dir) : Eigen::Vector3d::Zero());
    }
    if (b.p.size() < 2)
      b.p.push_back(p), b.v.push_back(Eigen::Vector3d::Zero()), b.a.push_back(Eigen::Vector3d::Zero());
    cur_ = b;
    braking_ = true;
    res.outcome = Outcome::Brake;
    return res;
  }
  res.outcome = Outcome::Keep;
  return res;
}

bool LocalPlanner::roomy(const Eigen::Vector3d &q) const
{
  if (map_.query(q) != 0)
    return false;
  for (int k = 0; k < 8; k++)
  {
    const double th = k * M_PI / 4;
    if (map_.query(q + P_.goal_margin_m * Eigen::Vector3d(std::cos(th), std::sin(th), 0.0)) != 0)
      return false;
  }
  return true;
}

bool LocalPlanner::plan(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &a, double ts,
                        Reference &out, std::string &why)
{
  const double alt = goal_(2);
  Eigen::Vector3d d = goal_ - p;
  d(2) = 0.0;
  Eigen::Vector3d lg = d.norm() > P_.horizon_m ? Eigen::Vector3d(p + d.normalized() * P_.horizon_m) : goal_;
  lg(2) = alt;
  if (!roomy(lg))
  {
    // nearest roomy point: back along the line, then sideways
    const Eigen::Vector3d u = d.norm() > 1e-6 ? Eigen::Vector3d(d.normalized()) : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d w(-u(1), u(0), 0.0);
    bool found = false;
    for (double back = 0.0; back < P_.horizon_m && !found; back += 0.5)
      for (double side : {0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 3.0, -3.0, 4.0, -4.0})
      {
        const Eigen::Vector3d q = lg - u * back + w * side;
        if (roomy(q) && (q - p).norm() > 1.0)
        {
          lg = q;
          found = true;
          break;
        }
      }
    if (!found)
      return why = "no free local goal", false;
  }
  const double res = map_.resolution();
  Eigen::Vector3d lo = p.cwiseMin(lg) - Eigen::Vector3d(6, 6, 0), hi = p.cwiseMax(lg) + Eigen::Vector3d(6, 6, 0);
  lo(2) = alt - P_.slab_m;
  hi(2) = alt + P_.slab_m;
  lo = lo.cwiseMax(map_.origin() + Eigen::Vector3d::Constant(res));
  hi = hi.cwiseMin(map_.origin() + map_.extent() - Eigen::Vector3d::Constant(res));
  Eigen::Vector3d ps = p;
  ps(2) = std::clamp(ps(2), lo(2) + 0.05, hi(2) - 0.05);
  if (map_.query(ps) != 0)
    return why = "start inside the grown map", false;
  std::vector<Eigen::Vector3d> route;
  if (!std::isfinite(sfc_gen::planPath<VoxelMap>(ps, lg, lo, hi, &map_, P_.rrt_timeout_s, route)) || route.size() < 2)
    return why = "no route", false;
  std::vector<Eigen::Vector3d> pts;
  map_.surface(lo - Eigen::Vector3d::Constant(1.0), hi + Eigen::Vector3d::Constant(1.0), pts);
  for (int sx : {-1, 1})  // FIRI needs at least one point: far corners, outside every box
    for (int sy : {-1, 1})
      pts.emplace_back(p + Eigen::Vector3d(sx * 200.0, sy * 200.0, 0.0));
  std::vector<Eigen::MatrixX4d> hpolys;
  sfc_gen::convexCover(route, pts, lo, hi, 7.0, 3.0, hpolys);
  sfc_gen::shortCut(hpolys);
  if (hpolys.empty())
    return why = "no corridor", false;
  Eigen::Matrix3d ini, fin;
  ini << p, v, a;
  fin << lg, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  Eigen::VectorXd mag(5), pen(5), phy(6);
  mag << P_.max_vel, P_.max_bodyrate, P_.max_tilt_rad, P_.thrust_min_n, P_.thrust_max_n;
  for (int i = 0; i < 5; i++)
    pen(i) = i < int(P_.chi.size()) ? P_.chi[i] : 1e4;
  phy << P_.mass_kg, 9.81, 0.0, 0.0, 0.0, 1e-4;
  gcopter::GCOPTER_PolytopeSFC opt;
  Trajectory<5> traj;
  if (!opt.setup(P_.weight_t, ini, fin, hpolys, INFINITY, P_.smoothing_eps, P_.integral_intervs, mag, pen, phy))
    return why = "optimiser setup", false;
  if (!std::isfinite(opt.optimize(traj, P_.rel_cost_tol)) || traj.getPieceNum() < 1)
    return why = "optimiser", false;
  out = Reference();
  out.t0 = ts;
  out.dt = 0.01;
  const double T = traj.getTotalDuration();
  for (double t = 0.0; t <= T + 1e-9; t += out.dt)
  {
    const double tt = std::min(t, T);
    out.p.push_back(traj.getPos(tt));
    out.v.push_back(traj.getVel(tt));
    out.a.push_back(traj.getAcc(tt));
  }
  if (!(out.duration() >= P_.min_plan_s))
    return why = "plan too short (" + std::to_string(out.duration()) + " s)", false;
  for (size_t i = 0; i < out.p.size(); i++)
    if (!out.p[i].allFinite() || !out.v[i].allFinite() || !out.a[i].allFinite())
      return why = "plan not finite", false;
  // the check: every check_dt_s against the grown map (the start may sit on its edge after a near miss)
  bool leaving = map_.query(out.p.front()) != 0;
  const int step = std::max(1, int(std::round(P_.check_dt_s / out.dt)));
  for (size_t i = 0; i < out.p.size(); i += step)
  {
    const bool occ = map_.query(out.p[i]) != 0;
    if (leaving)
    {
      leaving = occ;
      continue;
    }
    if (occ)
      return why = "plan meets the map at " + std::to_string(i * out.dt) + " s", false;
    if (out.v[i].norm() > 1.1 * P_.max_vel)
      return why = "plan over speed", false;
  }
  return true;
}

}  // namespace coybot_avoidance
