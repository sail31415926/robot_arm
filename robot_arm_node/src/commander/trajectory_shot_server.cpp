/**
 * @file trajectory_shot_server.cpp
 * @brief TrajectoryShotServer 实现 —— MOTION_LINEAR / MOTION_ORBIT
 *
 * LINEAR：PTP 到起点→dwell→plan_line_ruckig 笛卡尔直线→wait_at_pose[→直线返回]。
 * ORBIT：PTP 到起始球坐标→dwell→plan_orbit_ruckig 球面轨道→wait_at_pose[→原路返回]。
 * 开头那段 PTP 走 approach_speed()（不入画，与 goal.transition_speed 解耦），其余段走 goal 档位。
 * 运镜段均为密集笛卡尔路点 + 批量 IK（末端严格贴合几何路径）；sphere_to_pose 复用 motion 几何。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/trajectory_shot_server.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include <robot_arm_interfaces/msg/arm_status.hpp>

#include "robot_arm_node/commander/motion_policy.hpp"
#include "robot_arm_node/motion/geometry.hpp"
#include "robot_arm_node/tuning.hpp"

namespace robot_arm_node::commander
{

using ArmStatus = robot_arm_interfaces::msg::ArmStatus;

namespace
{
// 超时 / 反馈频率 / 起点停留来自 tuning::params()（arm_params.yaml）
constexpr double DEG2RAD = M_PI / 180.0;

// WaitOutcome → exit_reason 字符串（success 情形外）
/**
 * @brief 把等待结果映射为 exit_reason 字符串。
 *
 * @param o 等待结果。
 * @return 对应的 exit_reason（REACHED 之外都是失败原因）。
 */
const char * outcome_reason(WaitOutcome o)
{
  switch (o) {
    case WaitOutcome::REACHED:   return "reached";
    case WaitOutcome::SETTLED:   return "reached";   // 超时兜底成立：对上层就是到位
    case WaitOutcome::STOPPED:   return "stopped";
    case WaitOutcome::CANCELLED: return "cancelled";
    default:                     return "timeout";
  }
}

// 规划期 PlanResult → exit_reason 字符串。Unreachable 映射为 "unreachable"，
// 命中 commander 的干净分支（恢复 IDLE + ABORTED，秒回、不进 ERROR、不空等到位超时）；
// Error 保留原行为：与并发取消竞态时仍按 cancelled 归类。
/**
 * @brief 把规划期的 PlanResult 映射为 exit_reason 字符串。
 *
 * Unreachable 单独映射成 "unreachable"，命中 commander 的干净分支
 * （恢复 IDLE + ABORTED，秒回，不进 ERROR 也不空等到位超时）。Error 保留
 * 原行为：与并发取消存在竞态时仍按 cancelled 归类，避免把用户取消报成故障。
 *
 * @param pr 规划结果。
 * @param cancelled 当前是否处于取消/急停态（用于 Error 的归类）。
 * @return 对应的 exit_reason。
 */
const char * plan_exit_reason(motion::PlanResult pr, bool cancelled)
{
  switch (pr) {
    case motion::PlanResult::Cancelled:   return "cancelled";
    case motion::PlanResult::Unreachable: return "unreachable";
    default:                              return cancelled ? "cancelled" : "error";
  }
}

/**
 * @brief 取「搬到运镜起点」那一段的速度档位。
 *
 * 为什么不用 goal.transition_speed：这一段在 dwell_at_start 置 camera_ready 之前，
 * 根本不入画，慢没有任何画面收益。PTP 时长 = 位移/v_pos×1.5，SLOW 档（0.02m/s）
 * 搬 0.3m 就是 22.5s 纯空等，超过约 0.8m 直接顶穿 trajectory_shot 超时。
 * 档位键由 speed_profiles.approach_key 配置（默认 FAST）；运镜段与 return_to_start
 * 的返回段都在镜头里，仍用 goal 里的档位。
 *
 * @return 接近段的笛卡尔速度限制。
 */
const Speed & approach_speed()
{
  return speed_profile(tuning::params().approach_speed_key);
}
}  // namespace

/**
 * @brief 构造运镜 Action 服务端。
 *
 * @param node 宿主节点，用于取日志器。
 * @param motion 运动执行器，负责 IK、Ruckig 规划与轨迹下发。
 * @param status 状态聚合器，用于读位姿与上报起点就位/相机就绪标志。
 * @param monitor 执行监视器，负责等待到位与反馈上报。
 * @param is_stopped 急停判据回调。
 */
TrajectoryShotServer::TrajectoryShotServer(rclcpp::Node & node, MotionExecutor & motion,
                                           state::StatusAggregator & status,
                                           ExecutionMonitor & monitor,
                                           std::function<bool()> is_stopped)
: node_(node), logger_(node.get_logger()), motion_(motion), status_(status),
  monitor_(monitor), is_stopped_(std::move(is_stopped))
{
}

/**
 * @brief 执行 TrajectoryShot 动作，按 motion_type 分派到直线或球面轨道。
 *
 * @param gh Action 目标句柄。
 * @return Action 结果；motion_type 非法时返回 error。
 */
TrajectoryShotServer::Action::Result TrajectoryShotServer::execute(
    const std::shared_ptr<GoalHandle> & gh)
{
  const auto goal  = gh->get_goal();
  const Speed & speed = speed_profile(goal->transition_speed);

  if (goal->motion_type == Action::Goal::MOTION_LINEAR) {
    return execute_linear(gh, *goal, speed);
  }
  if (goal->motion_type == Action::Goal::MOTION_ORBIT) {
    return execute_orbit(gh, *goal, speed);
  }
  RCLCPP_ERROR(logger_, "非法 motion_type: %d", goal->motion_type);
  Action::Result r;
  r.success = false; r.exit_reason = "error"; r.error_code = ArmStatus::ERR_LIMIT;
  return r;
}

// ── MOTION_LINEAR ────────────────────────────────────────────────────────────────
/**
 * @brief 执行 MOTION_LINEAR 运镜：PTP 到起点 → 停顿 → 笛卡尔直线 → 可选原路返回。
 *
 * 步骤 0 的 can_plan_line 预判是关键：纯几何检查（不做 IK、不下发），规划注定
 * 失败时立刻拒绝，避免先花几十秒把机械臂搬到起点、再发现直线段走不通。
 *
 * 步骤 2 的直线以**指令起点**为基准而非实测位姿 —— 与 ORBIT 用指令球坐标一致；
 * 步骤 1 的到位等待已经把实测与指令的偏差压进到位容差内，用指令值能保证
 * 往返两次走的是同一条几何路径。
 *
 * 进度分配：无返回时 0→50→100，有返回时 0→33→67→100。
 *
 * @param gh Action 目标句柄。
 * @param goal Action 目标（起止位姿、是否返回）。
 * @param speed 运镜段的速度档位（搬到起点那段不用它，见 approach_speed）。
 * @return Action 结果。
 */
TrajectoryShotServer::Action::Result TrajectoryShotServer::execute_linear(
    const std::shared_ptr<GoalHandle> & gh, const Action::Goal & goal, const Speed & speed)
{
  const ArmPose & start = goal.linear_start_pose;
  const ArmPose & end   = goal.linear_end_pose;
  RCLCPP_INFO(logger_, "LINEAR 起始=(%.3f,%.3f,%.3f) 终止=(%.3f,%.3f,%.3f)",
              start.x, start.y, start.z, end.x, end.y, end.z);

  // 步骤 0：预判起点→终点直线能否规划成功（纯几何 Ruckig 检查，无 IK / 无下发），
  // 规划注定失败时直接返回，避免先耗时把机械臂搬到起点再落空
  if (!motion_.can_plan_line(start, end, speed)) {
    RCLCPP_ERROR(logger_, "LINEAR 起点到终点无法规划成功，拒绝执行");
    Action::Result result;
    result.success     = false;
    result.exit_reason = "unreachable";
    result.error_code  = ArmStatus::ERR_LIMIT;   // error_code 用 ERR_*，与运镜段兜底分支一致
    return result;
  }
  RCLCPP_INFO(logger_, "LINEAR 已检查可以规划，起始=(%.3f,%.3f,%.3f) 终止=(%.3f,%.3f,%.3f)",
              start.x, start.y, start.z, end.x, end.y, end.z);

  // 用户取消或急停均视为应中止
  auto cancelled = [this, gh]() { return gh->is_canceling() || (is_stopped_ && is_stopped_()); };

  Action::Result result;

  // 步骤 1：PTP 移到起始位姿（去程不要求直线；不入画，走 approach_speed 而非运镜档位）
  auto r = move_and_wait(gh, start, approach_speed(), "LINEAR 起始位",
                         0.0, goal.return_to_start ? 33.0 : 50.0);
  if (!r.success) return r;
  if (!dwell_at_start(gh, "LINEAR")) {
    result.exit_reason = "cancelled";
    return result;
  }

  // 步骤 2：笛卡尔直线 起点→终点（Ruckig 路点流 + 批量 IK，末端严格走直线；
  // 直线以指令起点为基准 —— 与 ORBIT 用指令球坐标一致，步骤 1 已把误差压进到位容差）
  if (cancelled()) { motion_.stop(); result.exit_reason = "cancelled"; return result; }
  RCLCPP_INFO(logger_, "LINEAR 直线运镜开始（Ruckig 直线路点流）");
  std::vector<double> seg_joints;   // 该段轨迹末点的关节解（到位判据用）
  {
    const auto pr = motion_.plan_line_ruckig(start, end, speed, cancelled, &seg_joints);
    if (pr != motion::PlanResult::Success) {
      result.success     = false;
      result.exit_reason = plan_exit_reason(pr, cancelled());
      result.error_code  = (pr == motion::PlanResult::Unreachable)
                               ? ArmStatus::ERR_LIMIT : ArmStatus::ERR_DRIVER;
      return result;
    }
  }
  const double p2_end = goal.return_to_start ? 67.0 : 100.0;
  r = wait_at_pose(gh, end, seg_joints, "LINEAR 终止到位",
                   goal.return_to_start ? 33.0 : 50.0, p2_end);
  if (!r.success || !goal.return_to_start) return r;

  // 步骤 3：笛卡尔直线原路返回
  RCLCPP_INFO(logger_, "LINEAR return_to_start: 直线返回起始位姿");
  if (cancelled()) { motion_.stop(); result.exit_reason = "cancelled"; return result; }
  std::vector<double> back_joints;
  {
    const auto pr = motion_.plan_line_ruckig(end, start, speed, cancelled, &back_joints);
    if (pr != motion::PlanResult::Success) {
      result.success     = false;
      result.exit_reason = plan_exit_reason(pr, cancelled());
      result.error_code  = (pr == motion::PlanResult::Unreachable)
                               ? ArmStatus::ERR_LIMIT : ArmStatus::ERR_DRIVER;
      return result;
    }
  }
  return wait_at_pose(gh, start, back_joints, "LINEAR 返回到位", 67.0, 100.0);
}

// ── MOTION_ORBIT ─────────────────────────────────────────────────────────────────
/**
 * @brief 执行 MOTION_ORBIT 运镜：PTP 到起始球坐标 → 停顿 → 球面轨道 → 可选原路返回。
 *
 * 球坐标（方位角/俯仰角/半径）在此处从度换算成弧度后下传，相机全程朝向球心。
 * 与 LINEAR 不同，这里没有 can_plan_line 那样的预判 —— 球面轨道的可达性依赖
 * 整条路径上的 IK，无法用纯几何提前判定，只能靠规划过程中的 Unreachable 返回。
 *
 * 进度分配：无返回时 0→20→100，有返回时 0→30→60→90→100。
 *
 * @param gh Action 目标句柄。
 * @param goal Action 目标（球心、起止球坐标、是否返回）。
 * @param speed 运镜段的速度档位（位置与姿态两组约束都会用到；
 *              搬到起点那段不用它，见 approach_speed）。
 * @return Action 结果。
 */
TrajectoryShotServer::Action::Result TrajectoryShotServer::execute_orbit(
    const std::shared_ptr<GoalHandle> & gh, const Action::Goal & goal, const Speed & speed)
{
  const double ox = goal.orbit_center_x, oy = goal.orbit_center_y, oz = goal.orbit_center_z;
  const double az0 = goal.azimuth_start_deg * DEG2RAD, el0 = goal.elevation_start_deg * DEG2RAD;
  const double r0  = goal.radius_start_m;
  const double az1 = goal.azimuth_end_deg * DEG2RAD, el1 = goal.elevation_end_deg * DEG2RAD;
  const double r1  = goal.radius_end_m;

  RCLCPP_INFO(logger_, "ORBIT 球心=(%.3f,%.3f,%.3f)  起(%.1f°,%.1f°,%.3fm) → 终(%.1f°,%.1f°,%.3fm)",
              ox, oy, oz, goal.azimuth_start_deg, goal.elevation_start_deg, r0,
              goal.azimuth_end_deg, goal.elevation_end_deg, r1);

  // 归一化 Ruckig 参数在 plan_orbit_waypoints 内按位置/姿态两组行程分别算，取更严者，
  // 故整个 speed 直接下传（此前只传姿态分量，纯径向推拉会拿 rad/s 当 m/s 用）

  // 用户取消或急停均视为应中止
  auto cancelled = [this, gh]() { return gh->is_canceling() || (is_stopped_ && is_stopped_()); };

  Action::Result result;

  // 步骤 1：PTP 移到起始球坐标
  const ArmPose start_pose = sphere_to_pose(goal.azimuth_start_deg, goal.elevation_start_deg, r0, ox, oy, oz);
  auto r_ptp = move_and_wait(gh, start_pose, approach_speed(), "ORBIT PTP→起点",
                             0.0, goal.return_to_start ? 30.0 : 20.0,
                             goal.azimuth_start_deg, goal.elevation_start_deg, r0);
  if (!r_ptp.success) return r_ptp;
  if (!dwell_at_start(gh, "ORBIT")) { result.exit_reason = "cancelled"; return result; }

  // 步骤 2：Ruckig 1-DOF 球面轨道（起 → 终）
  if (cancelled()) { motion_.stop(); result.exit_reason = "cancelled"; return result; }
  RCLCPP_INFO(logger_, "ORBIT 球面轨道开始（Ruckig 1-DOF）");
  std::vector<double> seg_joints;   // 该段轨迹末点的关节解（到位判据用）
  {
    const auto pr = motion_.plan_orbit_ruckig(ox, oy, oz, az0, el0, r0, az1, el1, r1,
                                              speed, cancelled, &seg_joints);
    if (pr != motion::PlanResult::Success) {
      result.success = false;
      result.exit_reason = plan_exit_reason(pr, cancelled());
      result.error_code = (pr == motion::PlanResult::Unreachable)
                              ? ArmStatus::ERR_LIMIT : ArmStatus::ERR_DRIVER;
      return result;
    }
  }

  // 步骤 2 收尾：等去程轨迹真正执行完再往下走。
  // ★ 2026-09-08 修：这次等待原来只在 return_to_start=false 时做，true 时被整段跳过，
  //   是实机「环绕刚起步就瞬间弹到终点」的根因 ★
  //   solve_and_send 是把整条 JointTrajectory 一次性发给 JTC 就返回，**不等执行完**。
  //   少了这次等待，步骤 3 的返回轨迹会在去程只跑了几秒（≈返回段批量 IK 的耗时）时
  //   把去程顶掉；返回轨迹的首点是环绕**终点**，而臂此刻还在起点附近，JTC 只有
  //   START_BLEND_SEC(0.2s) 的融合窗口去够它 —— 表现就是一瞬间弹到终点、再慢慢转回来。
  //   LINEAR 一直有这次等待（见 execute_linear 步骤 2），ORBIT 这里对齐它。
  const ArmPose end_pose = sphere_to_pose(goal.azimuth_end_deg, goal.elevation_end_deg, r1, ox, oy, oz);
  auto r_fwd = wait_at_pose(gh, end_pose, seg_joints, "ORBIT 终止到位",
                            goal.return_to_start ? 30.0 : 60.0,
                            goal.return_to_start ? 60.0 : 100.0,
                            goal.azimuth_end_deg, goal.elevation_end_deg, r1);
  if (!r_fwd.success || !goal.return_to_start) return r_fwd;

  // 步骤 3：Ruckig 1-DOF 原路返回（终 → 起）
  if (cancelled()) { motion_.stop(); result.exit_reason = "cancelled"; return result; }
  RCLCPP_INFO(logger_, "ORBIT return_to_start: 原路返回");
  std::vector<double> back_joints;
  {
    const auto pr = motion_.plan_orbit_ruckig(ox, oy, oz, az1, el1, r1, az0, el0, r0,
                                              speed, cancelled, &back_joints);
    if (pr != motion::PlanResult::Success) {
      result.success = false;
      result.exit_reason = plan_exit_reason(pr, cancelled());
      result.error_code = (pr == motion::PlanResult::Unreachable)
                              ? ArmStatus::ERR_LIMIT : ArmStatus::ERR_DRIVER;
      return result;
    }
  }
  return wait_at_pose(gh, start_pose, back_joints, "ORBIT 返回到位", 90.0, 100.0,
                      goal.azimuth_start_deg, goal.elevation_start_deg, r0);
}

// ── 到达起始点后的停顿 ─────────────────────────────────────────────────────────────
/**
 * @brief 在起始点停顿指定时长，期间保持对急停与取消的响应。
 *
 * 停顿的作用是让机械臂静定、相机稳定曝光后再开始运镜（否则起手那几帧会带
 * 残余振动）。停顿期间置 at_pose_start / camera_ready 标志供外部感知。
 * 用 20ms 轮询而非一次性 sleep，保证急停能在一拍内响应。
 *
 * @param gh Action 目标句柄。
 * @param label 日志前缀（"LINEAR" / "ORBIT"）。
 * @return 完整停顿结束返回 true；被急停或取消打断返回 false。
 */
bool TrajectoryShotServer::dwell_at_start(const std::shared_ptr<GoalHandle> & gh, const char * label)
{
  status_.set_at_pose_start(true);
  status_.set_camera_ready(true);
  const double dwell = tuning::params().dwell_at_start_sec;
  RCLCPP_INFO(logger_, "%s 已到达起始点，停顿 %.1fs 后执行运镜", label, dwell);

  using clock = std::chrono::steady_clock;
  const auto t_end = clock::now() + std::chrono::duration<double>(dwell);
  bool cancelled = false;
  while (clock::now() < t_end) {
    if (is_stopped_ && is_stopped_()) {
      RCLCPP_INFO(logger_, "%s 起点停顿期间被急停", label);
      cancelled = true; break;
    }
    if (gh->is_canceling()) {
      motion_.stop();
      RCLCPP_INFO(logger_, "%s 起点停顿期间被取消", label);
      cancelled = true; break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(0.02));
  }
  status_.set_at_pose_start(false);
  return !cancelled;
}

// ── 共用：IK 移动 + 等待到位 ────────────────────────────────────────────────────────
/**
 * @brief PTP 移动到目标位姿并等待到位（IK + 单点轨迹）。
 *
 * 用于运镜前的「搬到起点」，不要求路径形状，只要求终点到位。
 *
 * @param gh Action 目标句柄。
 * @param target 目标位姿。
 * @param speed 速度档位。
 * @param label 日志前缀。
 * @param p_lo 本段进度下界（%）。
 * @param p_hi 本段进度上界（%）。
 * @param azimuth 反馈里回填的方位角（度），仅 ORBIT 用。
 * @param elevation 反馈里回填的俯仰角（度），仅 ORBIT 用。
 * @param radius 反馈里回填的半径（m），仅 ORBIT 用。
 * @return Action 结果。
 */
TrajectoryShotServer::Action::Result TrajectoryShotServer::move_and_wait(
    const std::shared_ptr<GoalHandle> & gh, const ArmPose & target, const Speed & speed,
    const char * label, double p_lo, double p_hi, double azimuth, double elevation, double radius)
{
  Action::Result result;

  auto exec_r = motion_.plan_and_execute(target, speed);
  if (!exec_r.success) {
    result.success = false; result.exit_reason = exec_r.exit_reason;
    result.error_code = ArmStatus::ERR_DRIVER;
    return result;
  }

  const ArmPose cur = status_.pose();
  const double dist = std::sqrt(std::pow(target.x - cur.x, 2) +
                                std::pow(target.y - cur.y, 2) +
                                std::pow(target.z - cur.z, 2));
  const double total_dur = std::max(dist / std::max(speed.v_pos, 1e-6), 0.5);

  ExecutionMonitor::WaitParams p;
  // 到位判据 = 臂 J1-3 到达 PTP 终点关节解（云台照常跟动但不进判据，理由见
  // commander/motion_policy.hpp 的 is_at_joints_prefix）
  p.arrived = arm_arrived_fn(exec_r.target_joints, target);
  p.settled = arm_settled_fn(exec_r.target_joints, label);
  p.is_cancel_requested = [gh]() { return gh->is_canceling(); };
  p.on_feedback = [this, gh, p_lo, p_hi, total_dur, azimuth, elevation, radius](double elapsed) {
    const double ratio = std::min(elapsed / std::max(total_dur, 1e-6), 0.999);
    auto fb = std::make_shared<Action::Feedback>();
    fb->progress_percent      = static_cast<float>(p_lo + ratio * (p_hi - p_lo));
    fb->elapsed_sec           = static_cast<float>(elapsed);
    fb->current_pose          = status_.pose();
    fb->current_azimuth_deg   = static_cast<float>(azimuth);
    fb->current_elevation_deg = static_cast<float>(elevation);
    fb->current_radius_m      = static_cast<float>(radius);
    gh->publish_feedback(fb);
  };
  p.timeout_sec = tuning::params().trajectory_shot_timeout_sec;
  p.feedback_hz = tuning::params().feedback_hz;
  p.label = std::string(label) + " ";
  const auto outcome = monitor_.wait_until(p);

  result.success     = is_reached(outcome);
  result.exit_reason = outcome_reason(outcome);
  result.error_code  = result.success ? ArmStatus::ERR_NONE : ArmStatus::ERR_TIMEOUT;
  return result;
}

// ── 等待已下发轨迹执行完毕（只轮询到位）────────────────────────────────────────────
/**
 * @brief 等待已下发的轨迹执行完毕（只轮询到位，不再下发指令）。
 *
 * 与 move_and_wait 的区别：轨迹已由 plan_line_ruckig / plan_orbit_ruckig 一次性
 * 下发完毕，这里没有「距离/速度」可以估时长，故进度分母沿用 0.1×timeout 这个
 * 经验归一化 —— 它只影响进度条读数，不影响实际等待与判定。
 *
 * @param gh Action 目标句柄。
 * @param target 目标位姿（退化判据用）。
 * @param target_joints 本段轨迹末点关节解。
 * @param label 日志前缀。
 * @param p_lo 本段进度下界（%）。
 * @param p_hi 本段进度上界（%）。
 * @param azimuth 反馈里回填的方位角（度）。
 * @param elevation 反馈里回填的俯仰角（度）。
 * @param radius 反馈里回填的半径（m）。
 * @return Action 结果。
 */
TrajectoryShotServer::Action::Result TrajectoryShotServer::wait_at_pose(
    const std::shared_ptr<GoalHandle> & gh, const ArmPose & target,
    const std::vector<double> & target_joints,
    const char * label, double p_lo, double p_hi, double azimuth, double elevation, double radius)
{
  ExecutionMonitor::WaitParams p;
  p.arrived = arm_arrived_fn(target_joints, target);
  p.settled = arm_settled_fn(target_joints, label);
  p.is_cancel_requested = [gh]() { return gh->is_canceling(); };
  const double timeout_ref = tuning::params().trajectory_shot_timeout_sec;
  p.on_feedback = [this, gh, p_lo, p_hi, azimuth, elevation, radius,
                  timeout_ref](double elapsed) {
    // 轨迹已下发、无 dist 依据；沿用原实现的 0.1×timeout 归一化
    const double ratio = std::min(elapsed / std::max(timeout_ref * 0.1, 1e-6), 0.999);
    auto fb = std::make_shared<Action::Feedback>();
    fb->progress_percent      = static_cast<float>(p_lo + ratio * (p_hi - p_lo));
    fb->elapsed_sec           = static_cast<float>(elapsed);
    fb->current_pose          = status_.pose();
    fb->current_azimuth_deg   = static_cast<float>(azimuth);
    fb->current_elevation_deg = static_cast<float>(elevation);
    fb->current_radius_m      = static_cast<float>(radius);
    gh->publish_feedback(fb);
  };
  p.timeout_sec = tuning::params().trajectory_shot_timeout_sec;
  p.feedback_hz = tuning::params().feedback_hz;
  p.label = std::string(label) + " ";
  const auto outcome = monitor_.wait_until(p);

  Action::Result result;
  result.success     = is_reached(outcome);
  result.exit_reason = outcome_reason(outcome);
  result.error_code  = result.success ? ArmStatus::ERR_NONE : ArmStatus::ERR_TIMEOUT;
  return result;
}

// ── 到位判据构造：只判臂 J1-3 ──────────────────────────────────────────────────────
/**
 * @brief 构造到位判据闭包：优先只判臂 J1-3，拿不到关节解时退化为笛卡尔判据。
 *
 * 末端 gimbal_tool0 在云台 J4-6 之后，云台回读不收敛会让笛卡尔判据永不满足，
 * 因此正常路径一律用关节前缀判据（见 CLAUDE.md）。退化分支带 WARN。
 *
 * @param target_joints 轨迹末点关节解，长度 ≥ 3 时走关节判据。
 * @param target 目标位姿，退化分支用。
 * @return 到位判据闭包。
 */
std::function<bool()> TrajectoryShotServer::arm_arrived_fn(
    const std::vector<double> & target_joints, const ArmPose & target)
{
  if (target_joints.size() >= motion::ARM_JOINT_COUNT) {
    return [this, target_joints]() {
      return is_at_joints_prefix(motion_.get_current_joints(), target_joints,
                                 motion::ARM_JOINT_COUNT);
    };
  }
  RCLCPP_WARN(logger_, "无末点关节解，退化为笛卡尔到位判据（云台未到位可能导致超时）");
  return [this, target]() { return is_at_pose(status_.pose(), target); };
}

// ── 超时兜底判据构造：只判臂 J1-3 ────────────────────────────────────────────────
/**
 * @brief 构造超时兜底判据：臂 J1-3 已静止且残差在放宽容差内（见 motion_policy.hpp）。
 *
 * 与 arm_arrived_fn 配对：同一组末点关节解，严格容差走 arrived，超时那一刻再用
 * 放宽容差问一次 settled。没有关节解就没有兜底（退化的笛卡尔判据本来就带 WARN）。
 *
 * @param target_joints 轨迹末点关节解，长度 ≥ 3 才构造。
 * @param label 日志前缀。
 * @return 兜底判据闭包；无关节解时为空。
 */
std::function<bool()> TrajectoryShotServer::arm_settled_fn(
    const std::vector<double> & target_joints, const char * label)
{
  if (target_joints.size() < motion::ARM_JOINT_COUNT) return nullptr;
  return monitor_.make_settled([this]() { return motion_.get_current_joints(); },
                               [this]() { return status_.joint_velocity_list(); },
                               target_joints, motion::ARM_JOINT_COUNT,
                               std::string(label) + " ");
}

// ── 球坐标 → Cartesian 位姿 ────────────────────────────────────────────────────────
/**
 * @brief 球坐标转末端位姿：位置取球面点，姿态取朝向球心的方向。
 *
 * 输入输出都用「度」（与 Action 接口一致），内部换成弧度调 motion 几何函数。
 *
 * @param azimuth_deg 方位角（度）。
 * @param elevation_deg 俯仰角（度）。
 * @param radius_m 半径（m）。
 * @param ox 球心 x（m）。
 * @param oy 球心 y（m）。
 * @param oz 球心 z（m）。
 * @return 对应的末端位姿（位置 m，姿态角为度）。
 */
ArmPose TrajectoryShotServer::sphere_to_pose(double azimuth_deg, double elevation_deg,
                                             double radius_m, double ox, double oy, double oz)
{
  const double az = azimuth_deg * DEG2RAD, el = elevation_deg * DEG2RAD;
  const auto c = motion::sphere_to_cart(az, el, radius_m, ox, oy, oz);
  const auto q = motion::aim_quat(c[0], c[1], c[2], ox, oy, oz);
  const auto rpy = motion::quat_to_rpy(q[0], q[1], q[2], q[3]);
  ArmPose pose;
  pose.x = static_cast<float>(c[0]); pose.y = static_cast<float>(c[1]); pose.z = static_cast<float>(c[2]);
  pose.roll  = static_cast<float>(rpy[0] / DEG2RAD);
  pose.pitch = static_cast<float>(rpy[1] / DEG2RAD);
  pose.yaw   = static_cast<float>(rpy[2] / DEG2RAD);
  return pose;
}

}  // namespace robot_arm_node::commander
