/**
 * @file track_target_server.cpp
 * @brief TrackTargetServer 实现 —— IBVS 目标跟随控制循环
 *
 * SetParameters 启动 IBVS（paused=false）→ 主循环 10Hz：急停/取消/超时/特征丢失检查，
 * 图像+深度误差计算与收敛判定，发 Feedback；退出时 paused=true 挂起 IBVS 并调 goal 终态。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/commander/track_target_server.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

namespace robot_arm_node::commander
{

using namespace std::chrono_literals;
namespace
{
constexpr double IMG_STOP_TH     = 0.005;   // 图像误差收敛阈值（归一化）
constexpr double DEPTH_STOP_TH   = 0.02;    // 深度误差收敛阈值（m）
constexpr double FEATURE_TIMEOUT = 0.5;     // 特征丢失判定时长（s）
constexpr double FEEDBACK_HZ     = 10.0;
constexpr double DEFAULT_DEPTH   = 0.3;     // 未指定 desired_depth 时的默认值（m）
const char * IBVS_NODE     = "visp_ibvs_node";
const char * FEATURE_TOPIC = "/red_detector/feature";

/**
 * @brief 取单调时钟的当前秒数。
 *
 * 用 steady_clock 而非 ROS 时钟：本循环是 wall timer 驱动，且仿真下
 * /clock 频率很低，用 ROS 时钟求时间差会系统性丢步（见 CLAUDE.md）。
 *
 * @return 自 steady_clock 纪元起的秒数。
 */
double now_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief 构造一个 bool 类型的 ROS 参数消息。
 *
 * @param name 参数名。
 * @param v 参数值。
 * @return 填好类型与值的 Parameter 消息。
 */
rcl_interfaces::msg::Parameter bool_param(const std::string & name, bool v)
{
  rcl_interfaces::msg::Parameter p;
  p.name = name;
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  p.value.bool_value = v;
  return p;
}
/**
 * @brief 构造一个 double 类型的 ROS 参数消息。
 *
 * @param name 参数名。
 * @param v 参数值。
 * @return 填好类型与值的 Parameter 消息。
 */
rcl_interfaces::msg::Parameter double_param(const std::string & name, double v)
{
  rcl_interfaces::msg::Parameter p;
  p.name = name;
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  p.value.double_value = v;
  return p;
}
}  // namespace

/**
 * @brief 构造目标跟随 Action 服务端，订阅特征话题并建立 IBVS 参数客户端。
 *
 * 深度初值给 DEFAULT_DEPTH 而非 0：0 会让首帧的深度误差算出 log(0) = -inf。
 *
 * @param node 宿主节点。
 * @param status 状态聚合器，用于上报跟随状态与读当前位姿。
 * @param is_stopped 急停判据回调。
 */
TrackTargetServer::TrackTargetServer(rclcpp::Node & node, state::StatusAggregator & status,
                                     std::function<bool()> is_stopped)
: node_(node), logger_(node.get_logger()), status_(status), is_stopped_(std::move(is_stopped))
{
  feat_z_ = DEFAULT_DEPTH;
  feat_sub_ = node_.create_subscription<geometry_msgs::msg::PointStamped>(
      FEATURE_TOPIC, 10,
      [this](const geometry_msgs::msg::PointStamped & msg) { this->on_feature(msg); });
  param_cli_ = node_.create_client<SetParameters>(std::string("/") + IBVS_NODE + "/set_parameters");
}

/**
 * @brief 特征话题回调：缓存目标在图像上的归一化坐标与深度。
 *
 * 丢弃非有限值与非正深度 —— 检测器在目标丢失时可能发 NaN/0，
 * 混进缓存会让误差计算和收敛判定失效。
 *
 * @param msg 特征点消息（x/y 为归一化图像坐标，z 为深度 m）。
 */
void TrackTargetServer::on_feature(const geometry_msgs::msg::PointStamped & msg)
{
  if (!std::isfinite(msg.point.z) || msg.point.z <= 0.0) return;
  std::lock_guard<std::mutex> lk(feat_mtx_);
  feat_x_ = msg.point.x; feat_y_ = msg.point.y; feat_z_ = msg.point.z;
  last_feat_s_ = now_s();
  has_feat_ = true;
}

/**
 * @brief 批量设置 IBVS 节点的参数。
 *
 * 参数服务不可用时返回 true（视为成功）—— 仿真或未起 visp_ibvs_node 的场景
 * 下不应因此让整个动作失败，只打一条 INFO。
 *
 * @param params 待设置的参数列表。
 * @return 全部设置成功（或服务不存在）返回 true，任一失败/超时返回 false。
 */
bool TrackTargetServer::set_params(const std::vector<rcl_interfaces::msg::Parameter> & params)
{
  if (!param_cli_->wait_for_service(1s)) {
    RCLCPP_INFO(logger_, "%s 参数服务不可用，仿真模式跳过", IBVS_NODE);
    return true;
  }
  auto req = std::make_shared<SetParameters::Request>();
  req->parameters = params;
  auto future = param_cli_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
    param_cli_->remove_pending_request(future);
    return false;
  }
  auto resp = future.get();
  if (!resp) return false;
  for (const auto & r : resp->results) {
    if (!r.successful) return false;
  }
  return true;
}

/**
 * @brief 挂起或恢复 IBVS 视觉伺服。
 *
 * @param paused true 挂起（停止发速度指令），false 恢复。
 * @return 设置成功返回 true。
 */
bool TrackTargetServer::set_paused(bool paused)
{
  return set_params({bool_param("paused", paused)});
}

/**
 * @brief 请求中止当前跟随（线程安全，供外部调用）。
 *
 * 只置标志位，由主循环下一拍检测并退出，不直接操作 Action 句柄。
 */
void TrackTargetServer::cancel()
{
  cancel_flag_.store(true);
}

/**
 * @brief 执行 TrackTarget 动作：启动 IBVS → 10Hz 监控循环 → 退出时挂起 IBVS。
 *
 * 本服务端自己不算控制量，速度指令由 visp_ibvs_node 直接发出；这里只负责
 * 启停 IBVS、监控误差与各种退出条件、上报 Feedback。循环内检查顺序为
 * 急停 → 取消 → 总超时 → 特征丢失 → 误差与收敛 → Feedback。
 *
 * hold_on_converge 为 true 时收敛也不退出（持续保持对准），此时只能靠取消、
 * 超时、特征丢失或急停结束动作。无论从哪条路径退出，都必须走到函数末尾的
 * 清理段把 IBVS 挂起 —— 否则视觉伺服会在动作结束后继续驱动机械臂。
 *
 * @param gh Action 目标句柄。
 * @return Action 结果（含 exit_code、最终误差与位姿）。
 */
TrackTargetServer::Action::Result TrackTargetServer::execute(const std::shared_ptr<GoalHandle> & gh)
{
  const auto goal = gh->get_goal();
  cancel_flag_.store(false);

  Action::Result result;
  double img_err = 999.0, depth_err = 999.0;

  // 配置并启动 IBVS
  const double desired_depth = goal->desired_depth > 0.0 ? goal->desired_depth : DEFAULT_DEPTH;
  std::vector<rcl_interfaces::msg::Parameter> params{
      bool_param("paused", false),
      double_param("desired_x", goal->desired_x),
      double_param("desired_y", goal->desired_y),
      double_param("desired_depth", desired_depth),
      bool_param("constrain_height", goal->constrain_height)};
  if (goal->constrain_height && goal->desired_height > 0.0) {
    params.push_back(double_param("desired_height", goal->desired_height));
  }

  if (!set_params(params)) {
    RCLCPP_ERROR(logger_, "IBVS 参数设置失败，无法启动跟随");
    result.success = false;
    result.exit_code = Action::Goal::EXIT_ERROR;
    result.exit_reason = "error";
    gh->abort(std::make_shared<Action::Result>(result));
    return result;
  }

  status_.set_tracking(true, 0.0, 0.0);
  RCLCPP_INFO(logger_, "目标跟随启动  depth=%.2fm  hold=%d  timeout=%.1fs",
              desired_depth, goal->hold_on_converge, goal->total_timeout_sec);

  // 主控制循环
  const double t_start = now_s();
  const double fb_period = 1.0 / FEEDBACK_HZ;
  double last_fb = 0.0;

  while (true) {
    const double now = now_s();
    const double elapsed = now - t_start;

    // 急停
    if (is_stopped_ && is_stopped_()) {
      RCLCPP_INFO(logger_, "目标跟随期间被急停，退出");
      result.exit_code = Action::Goal::EXIT_ERROR; result.exit_reason = "stopped";
      result.success = false; break;
    }
    // 取消
    if (gh->is_canceling() || cancel_flag_.load()) {
      RCLCPP_INFO(logger_, "目标跟随被取消");
      result.exit_code = Action::Goal::EXIT_CANCELLED; result.exit_reason = "cancelled";
      result.success = true; break;
    }
    // 总超时
    if (goal->total_timeout_sec > 0.0 && elapsed >= goal->total_timeout_sec) {
      RCLCPP_WARN(logger_, "目标跟随总超时（%.1fs）", goal->total_timeout_sec);
      result.exit_code = Action::Goal::EXIT_TIMEOUT; result.exit_reason = "timeout";
      result.success = false; break;
    }

    // 特征快照
    bool has_feat; double last_t, fx, fy, fz;
    {
      std::lock_guard<std::mutex> lk(feat_mtx_);
      has_feat = has_feat_; last_t = last_feat_s_;
      fx = feat_x_; fy = feat_y_; fz = feat_z_;
    }

    // 特征丢失
    if (has_feat && (now - last_t) > FEATURE_TIMEOUT) {
      RCLCPP_WARN(logger_, "特征丢失超时，退出跟随");
      result.exit_code = Action::Goal::EXIT_FEATURE_LOST; result.exit_reason = "feature_lost";
      result.success = false; break;
    }

    // 误差计算
    if (has_feat) {
      img_err   = std::hypot(fx - goal->desired_x, fy - goal->desired_y);
      depth_err = fz > 0.0 ? std::fabs(std::log(fz / desired_depth)) : 999.0;
    } else {
      img_err = depth_err = 999.0;
    }
    status_.set_tracking(true, img_err, depth_err);

    // 收敛检测
    const bool is_converged = img_err < IMG_STOP_TH && depth_err < DEPTH_STOP_TH;
    if (is_converged && !goal->hold_on_converge) {
      RCLCPP_INFO(logger_, "目标收敛  img_err=%.4f  depth_err=%.3fm", img_err, depth_err);
      result.exit_code = Action::Goal::EXIT_CONVERGED; result.exit_reason = "converged";
      result.success = true; break;
    }

    // Feedback
    if (now - last_fb >= fb_period) {
      auto fb = std::make_shared<Action::Feedback>();
      fb->img_err      = static_cast<float>(img_err);
      fb->depth_err_m  = static_cast<float>(depth_err);
      fb->elapsed_sec  = static_cast<float>(elapsed);
      fb->is_converged = is_converged;
      fb->current_pose = status_.pose();
      gh->publish_feedback(fb);
      last_fb = now;
    }

    std::this_thread::sleep_for(20ms);
  }

  // 退出清理：无论何种退出路径都必须挂起 IBVS，否则它会继续驱动机械臂
  // 退出清理
  set_paused(true);
  status_.set_tracking(false, 0.0, 0.0);

  result.final_img_err     = static_cast<float>(img_err);
  result.final_depth_err_m = static_cast<float>(depth_err);
  result.final_pose        = status_.pose();

  // 按退出原因调对应的终态方法：取消走 canceled，其余按成败 succeed/abort
  auto result_ptr = std::make_shared<Action::Result>(result);
  if (result.exit_code == Action::Goal::EXIT_CANCELLED) {
    gh->canceled(result_ptr);
  } else if (result.success) {
    gh->succeed(result_ptr);
  } else {
    gh->abort(result_ptr);
  }

  RCLCPP_INFO(logger_, "目标跟随结束  reason=%s  img_err=%.4f",
              result.exit_reason.c_str(), result.final_img_err);
  return result;
}

}  // namespace robot_arm_node::commander
