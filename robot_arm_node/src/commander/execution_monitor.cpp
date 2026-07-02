/**
 * @file execution_monitor.cpp
 * @brief ExecutionMonitor::wait_until 实现 —— 轮询等待到位
 *
 * 循环内检查顺序与 Python 一致：急停 → 取消（发 stop_motion）→ 反馈(节流) → 到位 → sleep。
 * 计时用 steady_clock；超时补一条 warn（Python 原实现静默）。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/execution_monitor.hpp"

#include <chrono>
#include <thread>

namespace robot_arm_node::commander
{

ExecutionMonitor::ExecutionMonitor(rclcpp::Logger logger,
                                   std::function<bool()> is_stopped,
                                   std::function<void()> stop_motion)
: logger_(std::move(logger)),
  is_stopped_(std::move(is_stopped)),
  stop_motion_(std::move(stop_motion))
{
}

WaitOutcome ExecutionMonitor::wait_until(const WaitParams & p)
{
  using clock = std::chrono::steady_clock;
  const auto t_start = clock::now();
  const bool do_fb   = static_cast<bool>(p.on_feedback) && p.feedback_hz > 0.0;
  const double fb_period = do_fb ? (1.0 / p.feedback_hz) : 0.0;
  double last_fb = 0.0;

  auto elapsed_s = [&]() {
    return std::chrono::duration<double>(clock::now() - t_start).count();
  };

  while (elapsed_s() < p.timeout_sec) {
    // 急停：运动已由 ArmStop 停止，立即退出（commander 保持 STOPPED）
    if (is_stopped_ && is_stopped_()) {
      RCLCPP_INFO(logger_, "%s执行期间被急停，中止", p.label.c_str());
      return WaitOutcome::STOPPED;
    }

    // 用户取消：停运动并退出（状态转换由 commander execute 回调统一处理）
    if (p.is_cancel_requested && p.is_cancel_requested()) {
      if (p.stop_motion_on_cancel && stop_motion_) stop_motion_();
      RCLCPP_INFO(logger_, "%s被取消", p.label.c_str());
      return WaitOutcome::CANCELLED;
    }

    const double now = elapsed_s();
    if (do_fb && (now - last_fb) >= fb_period) {
      p.on_feedback(now);
      last_fb = now;
    }

    if (p.arrived && p.arrived()) {
      RCLCPP_INFO(logger_, "%s到位", p.label.c_str());
      return WaitOutcome::REACHED;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(p.poll_dt));
  }

  // 超时补一条 warn（对应 Python：原实现静默，这里不再无声）
  RCLCPP_WARN(logger_, "%s等待超时（%.0fs）", p.label.c_str(), p.timeout_sec);
  return WaitOutcome::TIMEOUT;
}

}  // namespace robot_arm_node::commander
