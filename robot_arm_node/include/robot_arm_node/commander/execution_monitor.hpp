/**
 * @file execution_monitor.hpp
 * @brief 执行监视器（C++）—— 统一「下发轨迹后轮询等待到位」的循环骨架
 *
 * 对应 Python commander/execution_monitor.py。把 MoveToPose / TrajectoryShot / Homing 里
 * 几乎相同的「等待到位」轮询循环收成一份：急停 → 取消 → 反馈(节流) → 到位 → sleep。
 * 差异（到位判据 / Feedback 内容 / 超时·频率）作参数注入，不与具体 Action 类型耦合：
 * is_stopped / stop_motion / arrived / is_cancel_requested / on_feedback 均为 std::function。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace robot_arm_node::commander
{

// wait_until 的退出原因（对应 Python WaitOutcome）
enum class WaitOutcome
{
  REACHED,    // 到位（arrived() 返回 true）
  STOPPED,    // 执行期间被急停（is_stopped()）
  CANCELLED,  // 用户取消（is_cancel_requested()），已发 stop_motion()
  TIMEOUT,    // 超过 timeout_sec 仍未到位
  SETTLED,    // 超时那一刻 settled() 成立：臂已静止在放宽容差内，视为到位（见 WaitParams::settled）
};

// SETTLED 也算到位：调用方只看 success 与否，兜底成功的动作对上层就是 "reached"
inline bool is_reached(WaitOutcome o)
{
  return o == WaitOutcome::REACHED || o == WaitOutcome::SETTLED;
}

class ExecutionMonitor
{
public:
  // 轮询默认参数
  static constexpr double DEFAULT_POLL_DT     = 0.01;   // s（100Hz）
  static constexpr double DEFAULT_FEEDBACK_HZ = 10.0;

  // is_stopped：急停判据；stop_motion：取消时急停。二者由 commander 注入。
  ExecutionMonitor(rclcpp::Logger logger,
                   std::function<bool()> is_stopped,
                   std::function<void()> stop_motion);

  // 一次等待的全部参数（用聚合体替代 Python 的多默认位置参数，构造点更清晰）
  struct WaitParams
  {
    std::function<bool()>       arrived;                        // 到位判据（必填）
    // 超时兜底判据（可选）：只在 timeout 那一刻问一次。成立 → 返回 SETTLED（算到位），
    // 不成立 → TIMEOUT。用途：严格容差没满足但臂已静止在放宽容差内时不再误报 ERROR；
    // 判据本身见 motion_policy.hpp 的 is_settled_near_prefix。传空 = 纯 timeout 行为。
    std::function<bool()>       settled = nullptr;
    std::function<bool()>       is_cancel_requested = nullptr;  // 取消判据；服务场景传空=不检测
    std::function<void(double)> on_feedback = nullptr;          // 节流后回调(elapsed)，自建并发布 Feedback
    double timeout_sec = 0.0;
    double feedback_hz = DEFAULT_FEEDBACK_HZ;                   // <=0 不发反馈
    double poll_dt     = DEFAULT_POLL_DT;
    std::string label  = "";                                   // 日志前缀
    bool   stop_motion_on_cancel = true;
  };

  // 轮询等待，直到到位 / 急停 / 取消 / 超时（超时前先问一次 settled 兜底）。检查顺序与 Python 一致。
  WaitOutcome wait_until(const WaitParams & p);

  // 构造 WaitParams::settled 兜底判据：对前 n 轴做 is_settled_near_prefix，成立时把
  // 实际残差 / 速度打进日志（标定 tolerance.joint_rad 就靠这一行）。
  // get_pos / get_vel 返回 motion::JOINT_NAMES 顺序的关节位置 / 速度；target 长度 ≥ n。
  std::function<bool()> make_settled(std::function<std::vector<double>()> get_pos,
                                     std::function<std::vector<double>()> get_vel,
                                     std::vector<double> target, size_t n,
                                     std::string label) const;

private:
  rclcpp::Logger        logger_;
  std::function<bool()> is_stopped_;
  std::function<void()> stop_motion_;
};

}  // namespace robot_arm_node::commander
