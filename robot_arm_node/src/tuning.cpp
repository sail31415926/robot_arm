/**
 * @file tuning.cpp
 * @brief tuning::params() 的声明/读入与校验（取舍说明见 tuning.hpp）
 *
 * 这里只做「把 YAML 灌进结构体」这一件事：ROS 2 中没 declare_parameter
 * 过的参数， params YAML 里给了也会被静默忽略，所以每一项都必须在这里开出口子。
 * 真正调参改 robot_arm_bringup/config/arm_params.yaml，不要改本文件里的默认值
 * —— 那些默认值只是「没挂 YAML 也能跑」的兜底（取值 = 参数化之前各处 constexpr
 * 的原值）。
 *
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/tuning.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "robot_arm_node/motion/constants.hpp"

namespace robot_arm_node::tuning {

/**
 * @brief 根据档位键返回笛卡尔速度参数。
 * @param key 档位键，0 为慢速，2 为快速，其余值为正常速度。
 * @return 对应档位的速度参数。
 */
const Speed& Params::speed(uint8_t key) const {
    switch (key) {
        case 0:
            return speed_slow;
        case 2:
            return speed_fast;
        default:
            return speed_normal;  // 含 SPEED_NORMAL(1)
                                  // 与非法值，与参数化前行为一致
    }
}

/**
 * @brief 根据档位键返回关节速度。
 * @param key 档位键，0 为慢速，2 为快速，其余值为正常速度。
 * @return 关节速度，单位为 rad/s。
 */
double Params::joint_speed_rps(uint8_t key) const {
    switch (key) {
        case 0:
            return joint_speed_slow_rps;
        case 2:
            return joint_speed_fast_rps;
        default:
            return joint_speed_normal_rps;
    }
}

/**
 * @brief 获取全局调参参数实例。
 * @return 调参参数实例的引用。
 */
Params& params() {
    static Params p;
    return p;
}

namespace {

/**
 * @brief 声明并读取一个必须为正数的参数。
 * @param node 用于声明参数和输出警告的 ROS 2 节点。
 * @param name 参数名称。
 * @param def 参数默认值。
 * @return 配置值；配置值非正时返回默认值。
 */
/// 读一个正数参数。这些量全部必须为正，≤0 一定是配错了 —— 退回默认并告警，
/// 而不是让 0 传下去（0 容差 = 永远判不到位；0 时长 = 除零）。
double load_positive(rclcpp::Node& node, const std::string& name, double def) {
    const double v = node.declare_parameter(name, def);
    if (v > 0.0) return v;
    RCLCPP_WARN(node.get_logger(), "参数 %s = %.4f 非正，已退回默认值 %.4f",
                name.c_str(), v, def);
    return def;
}

/**
 * @brief 声明并读取一组速度档位参数。
 * @param node 用于声明参数和输出警告的 ROS 2 节点。
 * @param name 参数名称。
 * @param def 速度参数默认值。
 * @return 读取到的速度参数；配置格式或数值非法时返回默认值。
 */
/// 读一组速度档位（YAML 里是 6 元数组，顺序 v_pos a_pos j_pos v_ori a_ori
/// j_ori）
Speed load_speed(rclcpp::Node& node, const std::string& name,
                 const Speed& def) {
    const std::vector<double> d{def.v_pos, def.a_pos, def.j_pos,
                                def.v_ori, def.a_ori, def.j_ori};
    const auto v = node.declare_parameter(name, d);
    if (v.size() != 6) {
        RCLCPP_WARN(node.get_logger(),
                    "参数 %s 需要 6 个数（v_pos a_pos j_pos v_ori a_ori "
                    "j_ori），实际 %zu 个，"
                    "已退回默认值",
                    name.c_str(), v.size());
        return def;
    }
    for (size_t i = 0; i < 6; ++i) {
        if (v[i] <= 0.0) {
            RCLCPP_WARN(node.get_logger(),
                        "参数 %s 第 %zu 项 %.4f 非正，整组退回默认值",
                        name.c_str(), i, v[i]);
            return def;
        }
    }
    return Speed{v[0], v[1], v[2], v[3], v[4], v[5]};
}

/**
 * @brief 声明并读取一组关节角参数。
 * @param node 用于声明参数和输出警告的 ROS 2 节点。
 * @param name 参数名称。
 * @param def 关节角默认值。
 * @return 读取到的关节角；数量不匹配时返回默认值。
 */
/// 读一组关节角。个数必须与 JOINT_NAMES 一致，否则下发的轨迹点长度就不对了。
std::vector<double> load_joints(rclcpp::Node& node, const std::string& name,
                                const std::vector<double>& def) {
    const auto v = node.declare_parameter(name, def);
    if (v.size() != motion::JOINT_NAMES.size()) {
        RCLCPP_WARN(node.get_logger(),
                    "参数 %s 需要 %zu 个关节角，实际 %zu 个，已退回默认值",
                    name.c_str(), motion::JOINT_NAMES.size(), v.size());
        return def;
    }
    return v;
}

}  // namespace

/**
 * @brief 声明、读取并校验全部调参参数。
 * @param node 用于声明参数、读取配置和输出日志的 ROS 2 节点。
 */
void declare_and_load(rclcpp::Node& node) {
    Params& p = params();

    // ── 到位容差
    // ────────────────────────────────────────────────────────────────
    p.position_tolerance_m =
        load_positive(node, "tolerance.position_m", p.position_tolerance_m);
    p.orientation_tolerance_deg = load_positive(
        node, "tolerance.orientation_deg", p.orientation_tolerance_deg);
    p.joint_tolerance_rad =
        load_positive(node, "tolerance.joint_rad", p.joint_tolerance_rad);

    // ── 笛卡尔速度档位
    // ──────────────────────────────────────────────────────────
    p.speed_slow = load_speed(node, "speed_profiles.slow", p.speed_slow);
    p.speed_normal = load_speed(node, "speed_profiles.normal", p.speed_normal);
    p.speed_fast = load_speed(node, "speed_profiles.fast", p.speed_fast);

    // ── 关节空间档位
    // ────────────────────────────────────────────────────────────
    p.joint_speed_slow_rps =
        load_positive(node, "joint_speed.slow_rps", p.joint_speed_slow_rps);
    p.joint_speed_normal_rps =
        load_positive(node, "joint_speed.normal_rps", p.joint_speed_normal_rps);
    p.joint_speed_fast_rps =
        load_positive(node, "joint_speed.fast_rps", p.joint_speed_fast_rps);
    p.joint_min_duration_sec = load_positive(
        node, "joint_speed.min_duration_sec", p.joint_min_duration_sec);
    p.joint_max_duration_sec = load_positive(
        node, "joint_speed.max_duration_sec", p.joint_max_duration_sec);
    if (p.joint_max_duration_sec < p.joint_min_duration_sec) {
        RCLCPP_WARN(node.get_logger(),
                    "joint_speed.max_duration_sec(%.2f) < min(%.2f)，已互换",
                    p.joint_max_duration_sec, p.joint_min_duration_sec);
        std::swap(p.joint_max_duration_sec, p.joint_min_duration_sec);
    }

    // ── IK / 时序
    // ───────────────────────────────────────────────────────────────
    p.ik_timeout_s = load_positive(node, "ik.timeout_s", p.ik_timeout_s);
    p.ik_sample_dt = load_positive(node, "ik.sample_dt", p.ik_sample_dt);
    // 抽取比由采样步长推导，保证两者永远自洽（见
    // tuning.hpp：不开成独立参数的原因）
    p.ik_decimate = static_cast<int>(
        std::max<long>(1L, std::lround(p.ik_sample_dt / motion::STREAM_DT)));

    // ── action 超时与反馈
    // ───────────────────────────────────────────────────────
    p.feedback_hz = load_positive(node, "action.feedback_hz", p.feedback_hz);
    p.move_to_pose_timeout_sec = load_positive(
        node, "action.move_to_pose_timeout_sec", p.move_to_pose_timeout_sec);
    p.trajectory_shot_timeout_sec =
        load_positive(node, "action.trajectory_shot_timeout_sec",
                      p.trajectory_shot_timeout_sec);
    p.move_to_joint_timeout_margin_sec =
        load_positive(node, "action.move_to_joint_timeout_margin_sec",
                      p.move_to_joint_timeout_margin_sec);
    p.move_to_joint_timeout_floor_sec =
        load_positive(node, "action.move_to_joint_timeout_floor_sec",
                      p.move_to_joint_timeout_floor_sec);
    p.validity_wait_sec =
        load_positive(node, "action.validity_wait_sec", p.validity_wait_sec);

    // ── 固定动作
    // ────────────────────────────────────────────────────────────────
    p.stowed_duration_sec = load_positive(node, "posture.stowed_duration_sec",
                                          p.stowed_duration_sec);
    p.homing_duration_sec = load_positive(node, "posture.homing_duration_sec",
                                          p.homing_duration_sec);
    p.dwell_at_start_sec =
        load_positive(node, "posture.dwell_at_start_sec", p.dwell_at_start_sec);
    p.stowed_joints =
        load_joints(node, "posture.stowed_joints", p.stowed_joints);
    p.homing_joints =
        load_joints(node, "posture.homing_joints", p.homing_joints);

    RCLCPP_INFO(
        node.get_logger(),
        "tuning 已加载：容差 %.3fm/%.1f°/%.3frad  IK %.0fms(抽取 1/%d)  "
        "关节档位 %.2f/%.2f/%.2f rad/s  feedback %.0fHz",
        p.position_tolerance_m, p.orientation_tolerance_deg,
        p.joint_tolerance_rad, p.ik_timeout_s * 1e3, p.ik_decimate,
        p.joint_speed_slow_rps, p.joint_speed_normal_rps,
        p.joint_speed_fast_rps, p.feedback_hz);
}

}  // namespace robot_arm_node::tuning
