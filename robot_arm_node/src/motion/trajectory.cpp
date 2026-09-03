/**
 * @file trajectory.cpp
 * @brief decimate / build_joint_trajectory / solve_and_send 实现
 *
 * decimate 降采样；build_joint_trajectory 中央差分算关节速度构建消息（刻意不补加速度，
 * 见函数内注释）；solve_and_send 逐点 IK（种子延续 / 首帧零种子重试 / 失败沿用上帧）后，
 * 整条轨迹时间轴后移 START_BLEND_SEC 留出起步融合段（消化指令起点与实际位姿的到位容差
 * 偏差，避免起步速度尖峰——实机运镜段起步抖动的主因），一次性发布整条 JointTrajectory。
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
// 起步融合段时长：轨迹起点（JTC 侧的当前状态）→ 首个规划路点之间留出的过渡时间。
// JTC 用它柔性消化「实际位姿 ↔ 指令起点」的到位容差偏差（≤1cm/2°），避免压缩在
// 首个 10ms 段里造成起步速度尖峰。
// ★ 只后移时间轴，**不**自己插入「实测关节」当首点，见 solve_and_send 末尾注释。
constexpr double START_BLEND_SEC = 0.2;

// 连续 IK 无解阈值：达到即判定路径成片驶出可达域，提前放弃（不再空跑完剩余无解点，
// 每个无解点最坏要阻塞 2×ik_timeout）。低于此值的零星漏解仍沿用上帧容忍。
// 降采样后的路点粒度（decimate_k×STREAM_DT）下，连续这么多点无解已是明确的边界越界，
// 而非求解器偶发抖动。
constexpr int MAX_CONSEC_IK_FAIL = 10;
}  // namespace

/**
 * @brief 按固定步长对路点序列进行降采样。
 *
 * 以 k 为步长从头开始保留路点，确保末端点不会丢失，从而保持原始轨迹的
 * 起止位姿与总时长不变。
 *
 * @param pts 原始路点序列。
 * @param k 降采样步长；当 k <= 1 或路点数过少时不做降采样。
 * @return 降采样后的路点序列。
 */
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

/**
 * @brief 根据关节位置与时间戳构建 JointTrajectory 消息。
 *
 * 采用中央差分法计算每个路点的关节速度，端点速度强制为 0，以便让
 * Ruckig 等轨迹控制器按静止起止生成平滑轨迹；与速度/加速度耦合的
 * 五次样条方案相比，此处刻意只输出位置和速度，避免放大 IK 噪声。
 *
 * @param joint_pos 每个路点的关节位置，形状为 [N][DOF]。
 * @param joint_t 每个路点对应的时间戳（秒）。
 * @param joint_names 关节名称列表。
 * @param stamp 轨迹消息头的时间戳。
 * @return 组装好的 JointTrajectory 消息。
 */
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

/**
 * @brief 逐点求解 IK，并在满足可达性条件后一次性发布整条 JointTrajectory。
 *
 * 过程包括：
 * - 按 decimate_k 降采样原始运动路径；
 * - 逐点调用 IK，并使用上一解作为种子；
 * - 首帧失败时尝试零种子重试；
 * - 连续多点无解判定为不可达，零星漏解沿用上一帧；
 * - 最终时间轴整体后移 START_BLEND_SEC 预留起步融合时间；
 * - 仅在全部成功后发布一条完整轨迹。
 *
 * @param node ROS 节点对象，用于获取时钟与构建消息。
 * @param ik_client IK 求解服务客户端。
 * @param traj_pub 轨迹发布器。
 * @param all_pts_in 原始路径点集合。
 * @param seed 初始 IK 种子（用于首帧求解）。
 * @param joint_names 关节名称列表。
 * @param group MoveIt 规划组名称。
 * @param eef_link 末端执行器链接名称。
 * @param base_frame 参考坐标系。
 * @param decimate_k 路径降采样步长。
 * @param ik_timeout_s 每次 IK 求解超时时间（秒）。
 * @param stop_check 中止检测回调，返回 true 表示取消当前运镜。
 * @param logger 日志记录器。
 * @param final_joints 输出最终关节解，用于上层到位判定。
 * @return 规划结果枚举，表示成功、取消、不可达或错误。
 */
PlanResult solve_and_send(
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
    rclcpp::Logger logger,
    std::vector<double> * final_joints)
{
  if (all_pts_in.empty()) return PlanResult::Error;

  // 服务可用性只在批量开始检查一次（之后逐点用廉价 service_is_ready）
  if (!ik_client->wait_for_service(1s)) {
    RCLCPP_ERROR(logger, "solve_and_send: /compute_ik 服务不可用");
    return PlanResult::Error;
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

  // 可达性统计：区分「零星漏解（沿用上帧容忍）」与「成片/首末无解（判定不可达）」
  int    consec_fail    = 0;      // 当前连续无解计数
  size_t first_fail_idx = 0;      // 本段连续无解的起始 step
  bool   last_pt_failed = false;  // 最近处理的这一点是否无解

  for (size_t idx = 0; idx < all_pts.size(); ++idx) {
    if (stopped()) {
      RCLCPP_INFO(logger, "solve_and_send: 中止（cancel / 急停）");
      return PlanResult::Cancelled;
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
      consec_fail    = 0;
      last_pt_failed = false;
    } else if (!joint_pos.empty()) {
      // 零星漏解：沿用上一帧继续；成片连续无解：路径成片驶出可达域 → 判定不可达
      sol = joint_pos.back();
      if (consec_fail == 0) first_fail_idx = idx;
      ++consec_fail;
      last_pt_failed = true;
      RCLCPP_WARN(logger, "IK 失败 step=%zu err=%d  pos=(%.3f,%.3f,%.3f)，沿用上帧",
                  idx, res.error_code, w.x, w.y, w.z);
      if (consec_fail >= MAX_CONSEC_IK_FAIL) {
        const Waypoint & wf = all_pts[first_fail_idx];
        RCLCPP_ERROR(logger,
            "连续 %d 点 IK 无解（自 step=%zu pos=(%.3f,%.3f,%.3f) 起），"
            "判定目标超出可达域，放弃本段运镜",
            consec_fail, first_fail_idx, wf.x, wf.y, wf.z);
        return PlanResult::Unreachable;
      }
    } else {
      // 首帧（含零种子重试）即无解：起点不可达
      RCLCPP_ERROR(logger, "IK 首帧无解 err=%d  pos=(%.3f,%.3f,%.3f)，目标不可达",
                   res.error_code, w.x, w.y, w.z);
      return PlanResult::Unreachable;
    }

    cur_seed = sol;
    joint_pos.push_back(std::move(sol));
    joint_t.push_back(w.t);
  }

  // 末点无解（哪怕连续数未到阈值）：整段走不到终点 → 不可达，不下发退化轨迹
  if (last_pt_failed) {
    const Waypoint & wl = all_pts.back();
    RCLCPP_ERROR(logger, "运镜末点 IK 无解 pos=(%.3f,%.3f,%.3f)，终点不可达，放弃本段运镜",
                 wl.x, wl.y, wl.z);
    return PlanResult::Unreachable;
  }

  // 起步融合：整条时间轴后移 START_BLEND_SEC，把融合交给 JTC 自己做。
  //
  // ★ 为什么**不**在首点插入实测关节（2026-09-01 修，实机「起步抖两下」的根因）★
  // 曾经的写法是 joint_pos.insert(begin, seed) + joint_t.insert(begin, 0.0)，两个坑：
  //  1) time_from_start=0 的首点让 JTC 的「首点之前」融合窗口宽度为 0
  //     （Trajectory::sample 只在 sample_time < 首点时刻时才用 state_before_traj_msg_
  //     插值），轨迹一起跑指令位置就**阶跃**到 seed —— 而 JTC 此刻保持的指令是上一段
  //     PTP 的终点 IK(指令起点)，于是起步瞬间往回跳一个到位残差 Δ（J1-3 上限
  //     tolerance.joint_rad=0.02rad；J4-6 是云台滞后回读，没有上限）。这是第一抖。
  //  2) 该点参与 build_joint_trajectory 的中央差分，把「Ruckig 静止起点」（真实速度
  //     严格为 0）的速度算成 Δ/(2×START_BLEND_SEC 量级)：ik.sample_dt=0.03 时
  //     jvel[1]=Δ/0.24，Δ=0.02 → 0.083rad/s。0.2s 融合段把速度拉到这个值后，紧接着
  //     30ms 的路点段位移≈0（Ruckig 起步 s∝j·t³/6），三次样条只能急停+反向过冲。
  //     这是第二抖。0.2s 的融合段与 30ms 的路点间距本就不该进同一个差分。
  // 现在首点就是 Ruckig 的静止起点（build_joint_trajectory 给端点零速），JTC 用
  // state_before_traj_msg_（open_loop_control=false → 实测位置+实测速度）到它之间插
  // 三次样条：零速起、零速到，既无阶跃也无速度尖峰。seed 因此只作 IK 种子用，不再
  // 兼任下发的物理起点 —— 实测量绝不写进指令流。
  for (auto & t : joint_t) t += START_BLEND_SEC;

  auto msg = build_joint_trajectory(joint_pos, joint_t, joint_names,
                                    node.get_clock()->now());

  // 批量 IK 期间若已急停/取消，放弃下发整条轨迹
  if (stopped()) {
    RCLCPP_INFO(logger, "solve_and_send: 急停生效，放弃下发轨迹");
    return PlanResult::Cancelled;
  }
  traj_pub->publish(msg);

  // 末点关节解带给上层做到位判据
  if (final_joints) *final_joints = joint_pos.back();

  const double plan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_plan0).count();
  RCLCPP_INFO(logger,
              "solve_and_send: 下发 %zu 个路点（原 %zu，降采样 1/%d），时长=%.2fs，IK 规划耗时=%.0fms",
              joint_pos.size(), n_raw, decimate_k, joint_t.back(), plan_ms);
  return PlanResult::Success;
}

}  // namespace robot_arm_node::motion
