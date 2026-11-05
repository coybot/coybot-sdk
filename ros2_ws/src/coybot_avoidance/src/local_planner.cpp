// Copyright 2026 Coybot AI, Inc.
// SPDX-License-Identifier: Apache-2.0
#include "coybot_avoidance/local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <istream>
#include <ostream>

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

void VoxelMap::init(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi, double res, double inflate, int hit_count,
                    const LogOdds &l)
{
  res_ = res;
  o_ = lo;
  n_ = ((hi - lo) / res).cast<int>();
  const size_t N = size_t(n_.prod());
  lo_.assign(N, 0);
  grow_.assign(N, 0);
  seen_.assign(N, 0);
  frame_ = 0;
  const auto q = [](double x) { return int(std::lround(x * 16.0)); };
  hit_q_ = q(l.hit), miss_q_ = q(l.miss), min_q_ = std::max(-128, q(l.min)), max_q_ = std::min(127, q(l.max));
  occ_q_ = q((std::max(hit_count, 1) - 0.5) * l.hit);
  const int k = int(std::ceil(inflate / res));
  ball_.clear();
  for (int i = -k; i <= k; i++)
    for (int j = -k; j <= k; j++)
      for (int m = -k; m <= k; m++)
        if (Eigen::Vector3d(i, j, m).norm() * res <= inflate + 1e-9)
          ball_.emplace_back(i, j, m);
}

void VoxelMap::update(const Eigen::Vector3i &id, int dq)
{
  const size_t a = at(id);
  const bool was = occupied(a);
  lo_[a] = int8_t(std::clamp(int(lo_[a]) + dq, min_q_, max_q_));
  const bool now = occupied(a);
  if (was == now)
    return;
  for (const auto &d : ball_)
  {
    const Eigen::Vector3i q = id + d;
    if (!in(q))
      continue;
    uint16_t &g = grow_[at(q)];
    g = now ? uint16_t(g + 1) : uint16_t(g > 0 ? g - 1 : 0);
  }
}

void VoxelMap::hit(const Eigen::Vector3d &p)
{
  if (!ready() || !p.allFinite())
    return;
  const Eigen::Vector3i id = idx(p);
  if (in(id))
    update(id, hit_q_);
}

void VoxelMap::beginFrame()
{
  if (++frame_ == 0)  // the stamp wrapped: forget old stamps so they cannot match this frame
  {
    std::fill(seen_.begin(), seen_.end(), uint8_t(0));
    frame_ = 1;
  }
}

void VoxelMap::frameHit(const Eigen::Vector3d &p)
{
  if (!ready() || !p.allFinite())
    return;
  const Eigen::Vector3i id = idx(p);
  if (!in(id))
    return;
  const size_t a = at(id);
  if (seen_[a] == frame_)
    return;
  seen_[a] = frame_;
  update(id, hit_q_);
}

void VoxelMap::frameRay(const Eigen::Vector3d &from, const Eigen::Vector3d &to)
{
  if (!ready() || !from.allFinite() || !to.allFinite())
    return;
  // Amanatides & Woo voxel traversal from the camera to the end voxel (exclusive)
  const Eigen::Vector3d d = to - from;
  const double len = d.norm();
  if (len < 1e-6)
    return;
  Eigen::Vector3i id = idx(from);
  const Eigen::Vector3i end = idx(to);
  Eigen::Vector3i step;
  Eigen::Vector3d tmax, tdelta;
  for (int i = 0; i < 3; i++)
  {
    step(i) = d(i) > 0 ? 1 : (d(i) < 0 ? -1 : 0);
    const double bound = o_(i) + (id(i) + (step(i) > 0 ? 1 : 0)) * res_;
    tmax(i) = step(i) != 0 ? (bound - from(i)) / d(i) : INFINITY;
    tdelta(i) = step(i) != 0 ? res_ / std::abs(d(i)) : INFINITY;
  }
  for (int guard = 0; guard < 4 * int(len / res_) + 8 && id != end; guard++)
  {
    if (in(id))
    {
      const size_t a = at(id);
      if (seen_[a] != frame_)
      {
        seen_[a] = frame_;
        update(id, miss_q_);
      }
    }
    int k = 0;
    if (tmax(1) < tmax(k)) k = 1;
    if (tmax(2) < tmax(k)) k = 2;
    if (tmax(k) > 1.0)
      break;
    id(k) += step(k);
    tmax(k) += tdelta(k);
  }
}

int VoxelMap::query(const Eigen::Vector3d &p) const
{
  if (!p.allFinite())
    return 1;
  const Eigen::Vector3i id = idx(p);
  if (!in(id))
    return 1;
  const size_t a = at(id);
  return occupied(a) ? 1 : (grow_[a] > 0 ? 2 : 0);
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
        if (!occupied(at(id)) && grow_[at(id)] == 0)
          continue;
        for (const auto &d : nb)
        {
          const Eigen::Vector3i q = id + Eigen::Vector3i(d[0], d[1], d[2]);
          if (in(q) && !occupied(at(q)) && grow_[at(q)] == 0)
          {
            pts.push_back(centre(id));
            break;
          }
        }
      }
}

namespace
{
template <class T> void put(std::ostream &os, const T &x) { os.write(reinterpret_cast<const char *>(&x), sizeof(T)); }
template <class T> bool get(std::istream &is, T &x) { return bool(is.read(reinterpret_cast<char *>(&x), sizeof(T))); }
void putV(std::ostream &os, const Eigen::Vector3d &v) { for (int i = 0; i < 3; i++) put(os, v(i)); }
bool getV(std::istream &is, Eigen::Vector3d &v) { bool ok = true; for (int i = 0; i < 3; i++) ok = ok && get(is, v(i)); return ok; }
const uint32_t kMagic = 0x43415632;  // "CAV2": log-odds map
}  // namespace

void VoxelMap::save(std::ostream &os) const
{
  put(os, kMagic);
  putV(os, o_);
  put(os, res_);
  for (int i = 0; i < 3; i++) put(os, int32_t(n_(i)));
  for (int v : {hit_q_, miss_q_, min_q_, max_q_, occ_q_}) put(os, int32_t(v));
  put(os, int32_t(ball_.size()));
  for (const auto &b : ball_) for (int i = 0; i < 3; i++) put(os, int8_t(b(i)));
  put(os, uint64_t(lo_.size()));
  os.write(reinterpret_cast<const char *>(lo_.data()), std::streamsize(lo_.size()));
}

bool VoxelMap::load(std::istream &is)
{
  uint32_t m = 0;
  int32_t n[3], q[5], nb;
  uint64_t sz;
  if (!get(is, m) || m != kMagic || !getV(is, o_) || !get(is, res_))
    return false;
  for (int i = 0; i < 3; i++) if (!get(is, n[i])) return false;
  for (int i = 0; i < 5; i++) if (!get(is, q[i])) return false;
  if (!get(is, nb)) return false;
  ball_.resize(size_t(nb));
  for (auto &b : ball_)
    for (int i = 0; i < 3; i++) { int8_t x; if (!get(is, x)) return false; b(i) = x; }
  if (!get(is, sz)) return false;
  n_ = Eigen::Vector3i(n[0], n[1], n[2]);
  hit_q_ = q[0], miss_q_ = q[1], min_q_ = q[2], max_q_ = q[3], occ_q_ = q[4];
  lo_.resize(sz);
  if (!is.read(reinterpret_cast<char *>(lo_.data()), std::streamsize(sz)) || size_t(n_.prod()) != sz)
    return false;
  grow_.assign(sz, 0);
  seen_.assign(sz, 0);
  frame_ = 0;
  for (int z = 0; z < n_(2); z++)
    for (int y = 0; y < n_(1); y++)
      for (int x = 0; x < n_(0); x++)
      {
        const Eigen::Vector3i id(x, y, z);
        if (!occupied(at(id)))
          continue;
        for (const auto &d : ball_)
          if (in(id + d))
            grow_[at(id + d)]++;
      }
  return true;
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

void LocalPlanner::save(std::ostream &os, double now, const State &odom) const
{
  map_.save(os);
  putV(os, goal_);
  put(os, now);
  putV(os, odom.p);
  putV(os, odom.v);
  put(os, cur_.t0);
  put(os, cur_.dt);
  put(os, uint64_t(cur_.p.size()));
  for (size_t i = 0; i < cur_.p.size(); i++)
    putV(os, cur_.p[i]), putV(os, cur_.v[i]), putV(os, cur_.a[i]);
}

bool LocalPlanner::load(std::istream &is, double &now, State &odom)
{
  uint64_t n = 0;
  if (!map_.load(is) || !getV(is, goal_) || !get(is, now) || !getV(is, odom.p) || !getV(is, odom.v) ||
      !get(is, cur_.t0) || !get(is, cur_.dt) || !get(is, n))
    return false;
  cur_.p.resize(n), cur_.v.resize(n), cur_.a.resize(n);
  for (size_t i = 0; i < n; i++)
    if (!getV(is, cur_.p[i]) || !getV(is, cur_.v[i]) || !getV(is, cur_.a[i]))
      return false;
  have_goal_ = true;
  done_ = false;
  return true;
}

bool LocalPlanner::replan(double now, Reference &out, std::string &why, PlanDebug &dbg, bool stuck)
{
  last_ok_t_ = stuck ? now - P_.escape_after_s - 1.0 : now;
  escape_.clear();
  const double ts = now + P_.lead_s;
  Eigen::Vector3d p, v, a;
  cur_.eval(ts, p, v, a);
  dbg = PlanDebug();
  dbg_ = &dbg;
  bool ok = plan(p, v, a, ts, out, why, false);
  if (!ok && stuck)
  {
    std::string why_e;
    ok = plan(p, v, a, ts, out, why_e, true);
    if (!ok)
      why += "; escape: " + why_e;
  }
  dbg_ = nullptr;
  return ok;
}

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
  const auto depth = [&](int r, int c) {
    const uint8_t *row = d.data + size_t(r) * d.step;
    return d.float32 ? double(reinterpret_cast<const float *>(row)[c])
                     : 1e-3 * double(reinterpret_cast<const uint16_t *>(row)[c]);
  };
  const auto world = [&](int r, int c, double z) {
    return Eigen::Vector3d(d.R_world_cam * Eigen::Vector3d((c - d.cx) * z / d.fx, (r - d.cy) * z / d.fy, z) +
                           d.t_world_cam);
  };
  // Every skip_pixel-th pixel with a valid return within range and the altitude slab: the return marks its voxel,
  // and its ray clears the voxels it passes through (hits first, so a voxel hit in this frame is not also cleared).
  // A zero (invalid) pixel says nothing. Rays and returns must come from the same pixels: with rays from fewer
  // pixels than returns, noisy returns outnumber the clearing and phantoms stay (unit test NoisyWall...).
  map_.beginFrame();
  thread_local std::vector<Eigen::Vector3d> ends;
  ends.clear();
  const int k = std::max(1, P_.skip_pixel);
  for (int r = 0; r < d.height; r += k)
    for (int c = 0; c < d.width; c += k)
    {
      const double z = depth(r, c);
      if (!std::isfinite(z) || z < 0.2 || z >= P_.max_range_m)
        continue;
      const Eigen::Vector3d pw = world(r, c, z);
      if (pw(2) < z_lo || pw(2) > z_hi)
        continue;
      map_.frameHit(pw);
      ends.push_back(pw);
    }
  if (P_.clear_rays)
    for (const auto &e : ends)
      map_.frameRay(d.t_world_cam, e);
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
  if (!std::isfinite(last_ok_t_))
    last_ok_t_ = now;
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
  bool ok = plan(p, v, a, ts, r, res.detail, false);
  if (!ok && ts - P_.lead_s - last_ok_t_ >= P_.escape_after_s)
  {
    std::string why_e;
    ok = plan(p, v, a, ts, r, why_e, true);  // dead end: a local goal along a route to the goal itself
    if (!ok)
      res.detail += "; escape: " + why_e;
  }
  if (ok)
  {
    cur_ = r;
    last_ok_t_ = now;
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

bool LocalPlanner::escapeGoal(const Eigen::Vector3d &p, const Eigen::Vector3d &ps, double ts, Eigen::Vector3d &lg)
{
  // reuse the last escape route for escape_hold_s while it is still clear; otherwise search a new one
  bool blocked = escape_.empty() || ts - escape_t_ >= P_.escape_hold_s;
  for (size_t i = 1; i < escape_.size() && !blocked; i++)  // the map grew since: still clear?
  {
    const double L = (escape_[i] - escape_[i - 1]).norm();
    for (double s = 0.0; s <= L && !blocked; s += map_.resolution())
      blocked = map_.query(escape_[i - 1] + (escape_[i] - escape_[i - 1]) * (L > 1e-9 ? s / L : 0.0)) != 0;
  }
  if (blocked && map_.query(ps) == 0)
  {
    const double res = map_.resolution();
    Eigen::Vector3d lo = map_.origin() + Eigen::Vector3d::Constant(res), hi = map_.origin() + map_.extent() -
                                                                              Eigen::Vector3d::Constant(res);
    lo(2) = goal_(2) - P_.slab_m;
    hi(2) = goal_(2) + P_.slab_m;
    std::vector<Eigen::Vector3d> route;
    escape_.clear();
    if (std::isfinite(sfc_gen::planPath<VoxelMap>(ps, goal_, lo, hi, &map_, P_.escape_timeout_s, route)) &&
        route.size() >= 2 && (route.back() - goal_).norm() < 0.5 * res + 1e-6)
    {
      escape_ = route;
      escape_t_ = ts;
    }
  }
  else if (blocked)
    escape_.clear();
  if (escape_.empty())
    return false;
  // local goal: horizon_m along the escape route from its point nearest the vehicle, back to a roomy one
  size_t k = 0;
  double best = INFINITY;
  for (size_t i = 0; i < escape_.size(); i++)
    if ((escape_[i] - p).norm() < best)
      best = (escape_[i] - p).norm(), k = i;
  std::vector<Eigen::Vector3d> ahead;
  double s = 0.0;
  ahead.push_back(escape_[k]);
  for (size_t i = k + 1; i < escape_.size() && s < P_.horizon_m; i++)
  {
    const double L = (escape_[i] - escape_[i - 1]).norm();
    for (double u = 0.1; u <= L && s + u <= P_.horizon_m; u += 0.1)
      ahead.push_back(escape_[i - 1] + (escape_[i] - escape_[i - 1]) * (u / L));
    s += L;
  }
  for (int i = int(ahead.size()) - 1; i >= 0; i--)
    if ((ahead[i] - p).head<2>().norm() > 1.0 && roomy(ahead[i]))
    {
      lg = ahead[i];
      if (dbg_)
        dbg_->escape = escape_;
      return true;
    }
  return false;
}

bool LocalPlanner::plan(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &a, double ts,
                        Reference &out, std::string &why, bool escape)
{
  const double alt = goal_(2);
  Eigen::Vector3d d = goal_ - p;
  d(2) = 0.0;
  Eigen::Vector3d lg = d.norm() > P_.horizon_m ? Eigen::Vector3d(p + d.normalized() * P_.horizon_m) : goal_;
  lg(2) = alt;
  Eigen::Vector3d pe = p;
  pe(2) = std::clamp(pe(2), alt - P_.slab_m + 0.05, alt + P_.slab_m - 0.05);
  if (escape && !escapeGoal(p, pe, ts, lg))
    return why = "no route to the goal", false;
  if (!escape && !roomy(lg))
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
  if (dbg_)
    dbg_->start = p, dbg_->local_goal = lg;
  Eigen::Vector3d lo = p.cwiseMin(lg) - Eigen::Vector3d(6, 6, 0), hi = p.cwiseMax(lg) + Eigen::Vector3d(6, 6, 0);
  lo(2) = alt - P_.slab_m;
  hi(2) = alt + P_.slab_m;
  lo = lo.cwiseMax(map_.origin() + Eigen::Vector3d::Constant(res));
  hi = hi.cwiseMin(map_.origin() + map_.extent() - Eigen::Vector3d::Constant(res));
  Eigen::Vector3d ps = p;
  ps(2) = std::clamp(ps(2), lo(2) + 0.05, hi(2) - 0.05);
  if (map_.query(ps) != 0)
    return why = "start inside the grown map", false;
  // OMPL counts an approximate solution as solved, so planPath can return a route that stops short of the local
  // goal. A corridor along it cannot reach the goal, and GCOPTER, asked to end there, leaves free space (round S2:
  // the vehicle stuck in dense forest, every plan meeting the map at the same place).
  std::vector<Eigen::Vector3d> route;
  const auto reaches = [&]() { return route.size() >= 2 && (route.back() - lg).norm() < 0.5 * res + 1e-6; };
  const auto search = [&](double timeout) {
    route.clear();
    return std::isfinite(sfc_gen::planPath<VoxelMap>(ps, lg, lo, hi, &map_, timeout, route));
  };
  if (!search(P_.rrt_timeout_s) || !reaches())
  {
    const std::vector<Eigen::Vector3d> first = route;
    if (!(P_.rrt_retry_timeout_s > 0.0 && search(P_.rrt_retry_timeout_s) && reaches()))
    {
      if (route.size() < 2)
        route = first;
      // walk back along the partial route to the furthest roomy point more than 1 m away
      bool usable = false;
      Eigen::Vector3d q;
      int seg = -1;
      for (int i = int(route.size()) - 1; P_.partial_route_goal && i >= 1 && !usable; i--)
      {
        const double L = (route[i] - route[i - 1]).norm();
        for (double s = L; s >= 0.0 && !usable; s -= 0.1)
        {
          q = route[i - 1] + (route[i] - route[i - 1]) * (L > 1e-9 ? s / L : 0.0);
          usable = (q - p).head<2>().norm() > 1.0 && roomy(q);
          seg = i;
        }
      }
      if (!usable)
        return why = route.size() < 2 ? "no route" : "route stops short of the local goal", false;
      route.resize(size_t(seg) + 1);  // the corridor follows the route up to that point
      route.back() = q;
      lg = q;
      if (dbg_)
        dbg_->local_goal = lg;
    }
  }
  std::vector<Eigen::Vector3d> pts;
  map_.surface(lo - Eigen::Vector3d::Constant(1.0), hi + Eigen::Vector3d::Constant(1.0), pts);
  for (int sx : {-1, 1})  // FIRI needs at least one point: far corners, outside every box
    for (int sy : {-1, 1})
      pts.emplace_back(p + Eigen::Vector3d(sx * 200.0, sy * 200.0, 0.0));
  std::vector<Eigen::MatrixX4d> hpolys;
  sfc_gen::convexCover(route, pts, lo, hi, 7.0, 3.0, hpolys);
  sfc_gen::shortCut(hpolys);
  if (dbg_)
    dbg_->route = route, dbg_->corridor = hpolys;
  if (hpolys.empty())
    return why = "no corridor", false;
  Eigen::Matrix3d ini, fin;
  ini << p, v, a;
  fin << lg, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  // FIRI keeps the corridor off voxel centres; the check rejects any sample inside a voxel, and GCOPTER's corridor
  // constraint is a penalty. Moving each face inward gives the trajectory room; without it, plans near trunks met
  // the map partway along again and again (round G2: the drone stuck). The unshrunk corridor is the fallback.
  if (P_.corridor_margin_m > 0.0)
  {
    std::vector<Eigen::MatrixX4d> shrunk = hpolys;
    for (auto &h : shrunk)
    {
      const Eigen::ArrayXd n = h.leftCols<3>().rowwise().norm();
      h.array().colwise() /= n;
      h.col(3).array() += P_.corridor_margin_m;  // inside: A x + b <= 0
    }
    std::string why_m;
    if (record(optimise(shrunk, ini, fin, ts, out, why_m), P_.corridor_margin_m, out, why_m))
      return true;
    if (!record(optimise(hpolys, ini, fin, ts, out, why), 0.0, out, why))
      return why = why_m + " (with margin); " + why + " (without)", false;
    return true;
  }
  return record(optimise(hpolys, ini, fin, ts, out, why), 0.0, out, why);
}

bool LocalPlanner::record(bool ok, double margin, const Reference &r, const std::string &why) const
{
  if (dbg_)
  {
    PlanDebug::Attempt at;
    at.margin = margin;
    at.why = ok ? "" : why;
    if (ok || why.rfind("optimiser", 0) != 0)  // a failed setup or optimisation leaves r stale
      at.ref = r;
    for (size_t i = 0; !ok && i < r.p.size(); i++)
      if (map_.query(r.p[i]) != 0 && i > 0)
      {
        at.bad_sample = int(i);
        break;
      }
    dbg_->attempts.push_back(at);
  }
  return ok;
}

bool LocalPlanner::optimise(const std::vector<Eigen::MatrixX4d> &hpolys, const Eigen::Matrix3d &ini,
                            const Eigen::Matrix3d &fin, double ts, Reference &out, std::string &why) const
{
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
