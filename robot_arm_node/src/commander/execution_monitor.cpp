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
#include <utility>

#include "robot_arm_node/commander/motion_policy.hpp"

namespace robot_arm_node::commander
{

/**
 * @brief 构造执行监视器，注入日志器与急停/停止运动回调。
 *
 * @param logger 日志器（按值传入后 move，调用方可传临时对象）。
 * @param is_stopped 急停判据回调，返回 true 表示当前处于急停态。
 * @param stop_motion 停止运动回调，用户取消时按需调用。
 */
ExecutionMonitor::ExecutionMonitor(rclcpp::Logger logger,
                                   std::function<bool()> is_stopped,
                                   std::function<void()> stop_motion)
: logger_(std::move(logger)),
  is_stopped_(std::move(is_stopped)),
  stop_motion_(std::move(stop_motion))
{
}

/**
 * @brief 轮询等待动作完成，直到到位、取消、急停或超时（超时前先问一次 settled 兜底）。
 *
 * 循环内的检查顺序是有意为之（与 Python 版一致）：急停 → 用户取消 → 反馈上报
 * → 到位判据 → sleep。急停优先于取消，因为急停时运动已被 ArmStop 停下，不需要
 * 再发一次 stop_motion；到位判据放在反馈之后，保证最后一帧反馈一定发得出去。
 * 计时统一用 steady_clock，不受系统时间跳变影响。
 *
 * @param p 等待参数（超时、轮询间隔、反馈频率与各回调）。
 * @return 等待结果：REACHED / CANCELLED / STOPPED / SETTLED / TIMEOUT。
 */
WaitOutcome ExecutionMonitor::wait_until(const WaitParams & p)
{
  using clock = std::chrono::steady_clock;
  const auto t_start = clock::now();
  const bool do_fb   = static_cast<bool>(p.on_feedback) && p.feedback_hz > 0.0;
  const double fb_period = do_fb ? (1.0 / p.feedback_hz) : 0.0;
  double last_fb = 0.0;

  // 距开始经过的秒数（steady_clock，单调递增）
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

    // 反馈按 feedback_hz 节流，避免高频 publish 挤占 Action 通道
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

  // 超时兜底：严格判据一直没满足，但臂若已静止在放宽容差内，就不是故障而是容差偏紧
  // —— 按到位收尾，避免上层进 ERROR 逼用户 reset_error。判据只在这一刻问一次，
  // 不参与循环内的到位判定（正常路径仍以 arrived 的严格容差为准）。
  if (p.settled && p.settled()) {
    RCLCPP_WARN(logger_, "%s严格容差 %.0fs 未满足，但臂已静止在放宽容差内，按到位收尾"
                "（残差见上一行；持续出现请按它放宽 tolerance.joint_rad）",
                p.label.c_str(), p.timeout_sec);
    return WaitOutcome::SETTLED;
  }

  // 超时补一条 warn（对应 Python：原实现静默，这里不再无声）
  RCLCPP_WARN(logger_, "%s等待超时（%.0fs）", p.label.c_str(), p.timeout_sec);
  return WaitOutcome::TIMEOUT;
}

/**
 * @brief 构造超时兜底判据（WaitParams::settled）。
 *
 * 闭包内调 is_settled_near_prefix；无论成立与否都把前 n 轴的最大残差与最大速度
 * 打成 WARN —— 这一行就是标定 tolerance.joint_rad 的依据：兜底成立说明容差偏紧，
 * 不成立则说明臂真没到位（残差远超放宽值或仍在动）。
 *
 * @param get_pos 取当前关节位置（motion::JOINT_NAMES 顺序）。
 * @param get_vel 取当前关节速度（同顺序，rad/s）。
 * @param target 目标关节角，长度 ≥ n。
 * @param n 参与判定的关节个数（前缀长度）。
 * @param label 日志前缀。
 * @return 可直接赋给 WaitParams::settled 的闭包。
 */
std::function<bool()> ExecutionMonitor::make_settled(
    std::function<std::vector<double>()> get_pos,
    std::function<std::vector<double>()> get_vel,
    std::vector<double> target, size_t n, std::string label) const
{
  return [logger = logger_, get_pos = std::move(get_pos), get_vel = std::move(get_vel),
          target = std::move(target), n, label = std::move(label)]() {
    double max_err = 0.0, max_vel = 0.0;
    const bool ok = is_settled_near_prefix(get_pos(), get_vel(), target, n, &max_err, &max_vel);
    const auto & tp = tuning::params();
    RCLCPP_WARN(logger,
                "%s超时时刻：前 %zu 轴最大残差 %.4f rad（严格容差 %.4f，放宽 ×%.1f=%.4f）"
                "最大速度 %.4f rad/s（静止阈值 %.4f）→ 兜底%s",
                label.c_str(), n, max_err, tp.joint_tolerance_rad, tp.settle_factor,
                tp.settle_factor * tp.joint_tolerance_rad, max_vel, tp.settle_velocity_rad_s,
                ok ? "成立" : "不成立");
    return ok;
  };
}

}  // namespace robot_arm_node::commander
