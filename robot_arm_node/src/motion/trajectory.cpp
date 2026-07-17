/**
 * @file trajectory.cpp
 * @brief decimate / build_joint_trajectory / solve_and_send 实现
 *
 * decimate 降采样；build_joint_trajectory 中央差分算关节速度构建消息（刻意不补加速度，
 * 见函数内注释）；solve_and_send 逐点 IK（种子延续 / 首帧零种子重试 / 失败沿用上帧）后，
 * 首点前插入当前关节作起步融合段（消化指令起点与实际位姿的到位容差偏差，
 * 避免起步速度尖峰——实机运镜段起步抖动的主因），一次性发布整条 JointTrajectory。
 *
 * @version 1.1
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/motion/trajectory.hpp"

#include <chrono>
#include <cstdint>

#include <builtin_interfaces/msg/duration.hpp>

namespace robot_arm_node::motion
{

using namespace std::chrono_literals;

namespace
{
// 起步融合段时长：首点（当前关节）→ 首个规划路点之间留出的过渡时间。
// JTC 用它柔性消化「实际位姿 ↔ 指令起点」的到位容差偏差（≤1cm/2°），避免压缩在
// 首个 10ms 段里造成起步速度尖峰。
constexpr double START_BLEND_SEC = 0.2;
}  // namespace

std::vector<Waypoint> decimate(const std::vector<Waypoint> & pts, int k)
{
  if (k <= 1 || pts.size() <= 2) return pts;
  std::vector<Waypoint> out;
  for (size_t i = 0; i < pts.size(); i += static_cast<size_t>(k)) {
    out.push_back(pts[i]);
  }
  // 末点若未被采到（(size-1) 不是 k 的整数倍），补上，保证起止位姿/时长不变
  if ((pts.size() - 1) % static_cast<size_t>(k) != 0) {
    out.push_back(pts.back());
  }
  return out;
}

trajectory_msgs::msg::JointTrajectory build_joint_trajectory(
    const std::vector<std::vector<double>> & joint_pos,
    const std::vector<double> & joint_t,
    const std::vector<std::string> & joint_names,
    const builtin_interfaces::msg::Time & stamp)
{
  const size_t n = joint_pos.size();
  const size_t dof = joint_names.size();

  // 中央差分算关节速度（支持非均匀间距），端点为零（Ruckig 轨迹静止起止）。
  // 刻意只给位置+速度（JTC 三次样条）：Ruckig 轮廓在恒 jerk 段内位置本就是三次多项式，
  // 三次 Hermite 已近似最优；补中央差分加速度换五次样条经数值验证反而更差
  //（差分加速度自带 O(j·h) 误差 + IK 噪声 /h² 放大，五次样条被迫穿过带误差端点）。
  std::vector<std::vector<double>> jvel(n, std::vector<double>(dof, 0.0));
  for (size_t i = 1; i + 1 < n; ++i) {
    const double dt2 = joint_t[i + 1] - joint_t[i - 1];
    if (dt2 > 1e-9) {
      for (size_t j = 0; j < dof; ++j) {
        jvel[i][j] = (joint_pos[i + 1][j] - joint_pos[i - 1][j]) / dt2;
      }
    }
  }

  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = stamp;
  msg.joint_names = joint_names;
  msg.points.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = joint_pos[i];
    pt.velocities = jvel[i];
    const int64_t ns = static_cast<int64_t>(joint_t[i] * 1e9);
    builtin_interfaces::msg::Duration d;
    d.sec = static_cast<int32_t>(ns / 1000000000LL);
    d.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    pt.time_from_start = d;
    msg.points.push_back(std::move(pt));
  }
  return msg;
}

bool solve_and_send(
    rclcpp::Node & node,
    const rclcpp::Client<GetPositionIK>::SharedPtr & ik_client,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & traj_pub,
    const std::vector<Waypoint> & all_pts_in,
    const std::vector<double> & seed,
    const std::vector<std::string> & joint_names,
    const std::string & group,
    const std::string & eef_link,
    const std::string & base_frame,
    int decimate_k,
    double ik_timeout_s,
    const std::function<bool()> & stop_check,
    rclcpp::Logger logger)
{
  if (all_pts_in.empty()) return false;

  // 服务可用性只在批量开始检查一次（之后逐点用廉价 service_is_ready）
  if (!ik_client->wait_for_service(1s)) {
    RCLCPP_ERROR(logger, "solve_and_send: /compute_ik 服务不可用");
    return false;
  }

  const auto stopped = [&stop_check]() -> bool {
    return stop_check && stop_check();
  };

  const size_t n_raw = all_pts_in.size();
  const std::vector<Waypoint> all_pts = decimate(all_pts_in, decimate_k);
  const auto t_plan0 = std::chrono::steady_clock::now();

  std::vector<double> cur_seed = seed;
  std::vector<std::vector<double>> joint_pos;
  std::vector<double> joint_t;
  joint_pos.reserve(all_pts.size());
  joint_t.reserve(all_pts.size());

  for (size_t idx = 0; idx < all_pts.size(); ++idx) {
    if (stopped()) {
      RCLCPP_INFO(logger, "solve_and_send: 中止（cancel / 急停）");
      return false;
    }

    const Waypoint & w = all_pts[idx];
    const auto ps = make_pose_stamped(
        w.x, w.y, w.z, w.qx, w.qy, w.qz, w.qw, base_frame, node.get_clock()->now());
    IkResult res = solve_ik(ik_client, ps, cur_seed, joint_names, group, eef_link,
                            ik_timeout_s, /*wait_service=*/false, logger);

    // 首帧失败时用零种子重试
    if (!res.joints && idx == 0) {
      res = solve_ik(ik_client, ps, std::vector<double>(joint_names.size(), 0.0),
                     joint_names, group, eef_link, ik_timeout_s,
                     /*wait_service=*/false, logger);
    }

    std::vector<double> sol;
    if (res.joints) {
      sol = std::move(*res.joints);
    } else if (!joint_pos.empty()) {
      // 降级：沿用上一帧
      sol = joint_pos.back();
      RCLCPP_WARN(logger, "IK 失败 step=%zu err=%d  pos=(%.3f,%.3f,%.3f)，沿用上帧",
                  idx, res.error_code, w.x, w.y, w.z);
    } else {
      RCLCPP_ERROR(logger, "IK 首帧失败 err=%d  pos=(%.3f,%.3f,%.3f)，放弃",
                   res.error_code, w.x, w.y, w.z);
      return false;
    }

    cur_seed = sol;
    joint_pos.push_back(std::move(sol));
    joint_t.push_back(w.t);
  }

  // 起步融合：首点插入当前关节（IK 种子 = 规划时刻的实测关节），其余整体后移
  // START_BLEND_SEC，由 JTC 在融合段内柔性对齐指令起点，消除起步速度尖峰
  joint_pos.insert(joint_pos.begin(), seed);
  joint_t.insert(joint_t.begin(), 0.0);
  for (size_t i = 1; i < joint_t.size(); ++i) joint_t[i] += START_BLEND_SEC;

  auto msg = build_joint_trajectory(joint_pos, joint_t, joint_names,
                                    node.get_clock()->now());

  // 批量 IK 期间若已急停/取消，放弃下发整条轨迹
  if (stopped()) {
    RCLCPP_INFO(logger, "solve_and_send: 急停生效，放弃下发轨迹");
    return false;
  }
  traj_pub->publish(msg);

  const double plan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_plan0).count();
  RCLCPP_INFO(logger,
              "solve_and_send: 下发 %zu 个路点（原 %zu，降采样 1/%d），时长=%.2fs，IK 规划耗时=%.0fms",
              joint_pos.size(), n_raw, decimate_k, joint_t.back(), plan_ms);
  return true;
}

}  // namespace robot_arm_node::motion
