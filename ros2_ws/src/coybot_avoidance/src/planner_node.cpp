// Copyright 2026 Coybot AI, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ROS 2 node around coybot_avoidance::LocalPlanner.
//
//   in   odom                (nav_msgs/Odometry: pose in world_frame with the vehicle's attitude, twist in the
//                             child (body) frame, as REP 105 / nav_msgs define it)
//        depth/image         (sensor_msgs/Image, 32FC1 in m or 16UC1 in mm)
//        depth/camera_info   (sensor_msgs/CameraInfo)
//        goal                (geometry_msgs/PoseStamped in world_frame; the altitude is flown)
//        TF base_frame -> camera optical frame (camera_frame, or the image's frame_id), looked up once
//   out  trajectory          (trajectory_msgs/MultiDOFJointTrajectory in world_frame: header.stamp is the start time
//                             on this node's clock, points every output_dt_s with position, velocity, acceleration;
//                             rotations are identity, the follower chooses yaw)
//        status              (std_msgs/String, one per planning cycle: "plan", "keep: why", "brake: why", ...)
#include <deque>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include "coybot_avoidance/local_planner.hpp"

namespace coybot_avoidance
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("avoidance_planner")
  {
    Params p;
    p.max_vel = declare_parameter("max_vel", p.max_vel);
    p.horizon_m = declare_parameter("horizon_m", p.horizon_m);
    p.lead_s = declare_parameter("lead_s", p.lead_s);
    p.map_res_m = declare_parameter("map_res", p.map_res_m);
    p.inflate_m = declare_parameter("inflate_m", p.inflate_m);
    p.hit_count = int(declare_parameter("hit_count", int64_t(p.hit_count)));
    p.max_range_m = declare_parameter("max_range_m", p.max_range_m);
    p.skip_pixel = int(declare_parameter("skip_pixel", int64_t(p.skip_pixel)));
    p.slab_m = declare_parameter("slab_m", p.slab_m);
    p.rrt_timeout_s = declare_parameter("rrt_timeout_s", p.rrt_timeout_s);
    p.check_dt_s = declare_parameter("check_dt_s", p.check_dt_s);
    p.min_plan_s = declare_parameter("min_plan_s", p.min_plan_s);
    p.goal_margin_m = declare_parameter("goal_margin_m", p.goal_margin_m);
    p.corridor_margin_m = declare_parameter("corridor_margin_m", p.corridor_margin_m);
    p.final_approach_m = declare_parameter("final_approach_m", p.final_approach_m);
    p.arrive_m = declare_parameter("arrive_m", p.arrive_m);
    p.brake_acc = declare_parameter("brake_acc", p.brake_acc);
    p.brake_latency_s = declare_parameter("brake_latency_s", p.brake_latency_s);
    p.brake_margin_m = declare_parameter("brake_margin_m", p.brake_margin_m);
    p.max_tilt_rad = declare_parameter("max_tilt_rad", p.max_tilt_rad);
    p.max_bodyrate = declare_parameter("max_bodyrate", p.max_bodyrate);
    p.mass_kg = declare_parameter("mass_kg", p.mass_kg);
    p.thrust_min_n = declare_parameter("thrust_min_n", p.thrust_min_n);
    p.thrust_max_n = declare_parameter("thrust_max_n", p.thrust_max_n);
    p.weight_t = declare_parameter("weight_t", p.weight_t);
    p.chi = declare_parameter("chi", p.chi);
    p.smoothing_eps = declare_parameter("smoothing_eps", p.smoothing_eps);
    p.integral_intervs = int(declare_parameter("integral_intervs", int64_t(p.integral_intervs)));
    p.rel_cost_tol = declare_parameter("rel_cost_tol", p.rel_cost_tol);
    p.map_margin_xy_m = declare_parameter("map_margin_xy_m", p.map_margin_xy_m);
    p.map_top_m = declare_parameter("map_top_m", p.map_top_m);
    world_frame_ = declare_parameter("world_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    camera_frame_ = declare_parameter("camera_frame", std::string(""));
    output_dt_ = declare_parameter("output_dt_s", 0.02);
    const double hz = declare_parameter("plan_hz", 10.0);
    planner_ = std::make_unique<LocalPlanner>(p);

    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);
    traj_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("trajectory", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdom(*m); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", 10, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { onInfo(*m); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr m) { onDepth(*m); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { onGoal(*m); });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "world %s, base %s, speed cap %.2f m/s, keep-out %.2f m", world_frame_.c_str(),
                base_frame_.c_str(), p.max_vel, p.inflate_m);
  }

private:
  struct OdomSample
  {
    double t;
    Eigen::Vector3d p, v;
    Eigen::Matrix3d R;
  };

  static double stamp_s(const builtin_interfaces::msg::Time &t) { return t.sec + 1e-9 * t.nanosec; }

  void onOdom(const nav_msgs::msg::Odometry &m)
  {
    OdomSample s;
    s.t = stamp_s(m.header.stamp);
    s.p << m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z;
    const auto &q = m.pose.pose.orientation;
    s.R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    s.v = s.R * Eigen::Vector3d(m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z);
    std::lock_guard<std::mutex> lk(mu_);
    odom_.push_back(s);
    while (odom_.size() > 400)
      odom_.pop_front();
  }

  void onInfo(const sensor_msgs::msg::CameraInfo &m)
  {
    std::lock_guard<std::mutex> lk(mu_);
    fx_ = m.k[0], fy_ = m.k[4], cx_ = m.k[2], cy_ = m.k[5];
    have_info_ = fx_ > 0 && fy_ > 0;
  }

  bool extrinsic(const std::string &image_frame)
  {
    if (have_ext_)
      return true;
    const std::string cam = camera_frame_.empty() ? image_frame : camera_frame_;
    try
    {
      const auto t = tf_buf_->lookupTransform(base_frame_, cam, tf2::TimePointZero);
      const auto &q = t.transform.rotation;
      R_base_cam_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      t_base_cam_ << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;
      have_ext_ = true;
      RCLCPP_INFO(get_logger(), "camera %s in %s: t [%.3f %.3f %.3f] m", cam.c_str(), base_frame_.c_str(),
                  t_base_cam_(0), t_base_cam_(1), t_base_cam_(2));
    }
    catch (const tf2::TransformException &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no TF %s -> %s yet: %s", base_frame_.c_str(),
                           cam.c_str(), e.what());
    }
    return have_ext_;
  }

  bool odomAt(double t, OdomSample &out) const
  {
    if (odom_.empty())
      return false;
    auto it = std::lower_bound(odom_.begin(), odom_.end(), t,
                               [](const OdomSample &a, double tt) { return a.t < tt; });
    if (it == odom_.end())
      it = std::prev(it);
    else if (it != odom_.begin() && std::abs(std::prev(it)->t - t) < std::abs(it->t - t))
      it = std::prev(it);
    if (std::abs(it->t - t) > 0.05)
      return false;
    out = *it;
    return true;
  }

  void onDepth(const sensor_msgs::msg::Image &m)
  {
    if (m.encoding != "32FC1" && m.encoding != "16UC1")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth encoding %s not supported", m.encoding.c_str());
      return;
    }
    if (!extrinsic(m.header.frame_id))
      return;
    std::lock_guard<std::mutex> lk(mu_);
    if (!have_info_ || !planner_->hasGoal())
      return;
    OdomSample s;
    if (!odomAt(stamp_s(m.header.stamp), s))
      return;
    DepthView d;
    d.data = m.data.data();
    d.width = int(m.width);
    d.height = int(m.height);
    d.step = m.step;
    d.float32 = m.encoding == "32FC1";
    d.fx = fx_, d.fy = fy_, d.cx = cx_, d.cy = cy_;
    d.R_world_cam = s.R * R_base_cam_;
    d.t_world_cam = s.p + s.R * t_base_cam_;
    planner_->insertDepth(d);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped &m)
  {
    if (!m.header.frame_id.empty() && m.header.frame_id != world_frame_)
    {
      RCLCPP_ERROR(get_logger(), "goal in frame %s, expected %s: ignored", m.header.frame_id.c_str(),
                   world_frame_.c_str());
      return;
    }
    std::lock_guard<std::mutex> lk(mu_);
    const Eigen::Vector3d g(m.pose.position.x, m.pose.position.y, m.pose.position.z);
    const Eigen::Vector3d start = odom_.empty() ? Eigen::Vector3d::Zero() : odom_.back().p;
    planner_->setGoal(g, start);
    RCLCPP_INFO(get_logger(), "goal [%.2f %.2f %.2f] m", g(0), g(1), g(2));
  }

  void publish(const Reference &r)
  {
    trajectory_msgs::msg::MultiDOFJointTrajectory msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp = rclcpp::Time(int64_t(r.t0 * 1e9), get_clock()->get_clock_type());
    msg.joint_names = {base_frame_};
    const double T = r.duration();
    const int n = std::max(2, int(std::ceil(T / output_dt_ - 1e-9)) + 1);
    Eigen::Vector3d p, v, a;
    for (int i = 0; i < n; i++)
    {
      const double t = std::min(i * output_dt_, T);
      r.eval(r.t0 + t, p, v, a);
      if (t >= T)  // eval reports rest after the end; keep the last sample's motion as planned
        p = r.p.back(), v = r.v.back(), a = r.a.back();
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint q;
      geometry_msgs::msg::Transform tf;
      tf.translation.x = p(0), tf.translation.y = p(1), tf.translation.z = p(2);
      tf.rotation.w = 1.0;
      geometry_msgs::msg::Twist tv, ta;
      tv.linear.x = v(0), tv.linear.y = v(1), tv.linear.z = v(2);
      ta.linear.x = a(0), ta.linear.y = a(1), ta.linear.z = a(2);
      q.transforms.push_back(tf);
      q.velocities.push_back(tv);
      q.accelerations.push_back(ta);
      q.time_from_start = rclcpp::Duration::from_seconds(t);
      msg.points.push_back(q);
    }
    traj_pub_->publish(msg);
  }

  void tick()
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (odom_.empty())
      return;
    State s;
    s.p = odom_.back().p;
    s.v = odom_.back().v;
    const bool was_done = planner_->arrived();
    const StepResult r = planner_->step(now().seconds(), s);
    if (r.outcome == Outcome::Plan || r.outcome == Outcome::Brake)
      publish(planner_->reference());
    if (r.outcome == Outcome::Idle || (was_done && r.outcome == Outcome::Arrived))
      return;
    std_msgs::msg::String st;
    st.data = std::string(to_string(r.outcome)) + (r.detail.empty() ? "" : ": " + r.detail);
    status_pub_->publish(st);
    if (r.outcome == Outcome::Brake || r.outcome == Outcome::Arrived)
      RCLCPP_INFO(get_logger(), "%s", st.data.c_str());
    else if (r.outcome != Outcome::Plan && r.detail != "final approach")
      RCLCPP_DEBUG(get_logger(), "%s", st.data.c_str());
  }

  std::mutex mu_;
  std::unique_ptr<LocalPlanner> planner_;
  std::deque<OdomSample> odom_;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, output_dt_ = 0.02;
  bool have_info_ = false, have_ext_ = false;
  Eigen::Matrix3d R_base_cam_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_base_cam_ = Eigen::Vector3d::Zero();
  std::string world_frame_, base_frame_, camera_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace coybot_avoidance

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor ex;
  auto node = std::make_shared<coybot_avoidance::PlannerNode>();
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
