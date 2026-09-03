/**
 * @file kinematics.cpp
 * @brief solve_ik / make_pose_stamped 实现 —— 异步 /compute_ik + 同步等待
 *
 * async_send_request 发 IK 请求后阻塞等 future（超时
 * remove_pending_request）；成功则按 joint_names 顺序重排关节解。wait_service
 * 区分单点(wait_for_service)与批量(service_is_ready)。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_node/motion/kinematics.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace robot_arm_node::motion {

using namespace std::chrono_literals;

/**
 * @brief 创建一个 PoseStamped 消息。
 *
 * @param x 目标位置 x 坐标。
 * @param y 目标位置 y 坐标。
 * @param z 目标位置 z 坐标。
 * @param qx 目标姿态四元数 x 分量。
 * @param qy 目标姿态四元数 y 分量。
 * @param qz 目标姿态四元数 z 分量。
 * @param qw 目标姿态四元数 w 分量。
 * @param frame_id 坐标系 ID。
 * @param stamp 时间戳。
 * @return 构造完成的 PoseStamped 对象。
 */
geometry_msgs::msg::PoseStamped make_pose_stamped(
    double x, double y, double z, double qx, double qy, double qz, double qw,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.header.stamp = stamp;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
    ps.pose.orientation.x = qx;
    ps.pose.orientation.y = qy;
    ps.pose.orientation.z = qz;
    ps.pose.orientation.w = qw;
    return ps;
}

/**
 * @brief 通过 MoveIt /compute_ik 服务求解逆运动学（IK）。
 *
 * 该函数会根据是否需要等待服务、请求超时以及目标姿态发出 IK 请求。
 * 成功时会按 joint_names 的顺序重排求解结果，并返回结果和错误码。
 *
 * @param client MoveIt IK 请求客户端。
 * @param pose_stamped 目标末端执行器位姿。
 * @param seed IK 初始关节角种子。
 * @param joint_names 机器人关节名称顺序。
 * @param group 运动组名称。
 * @param eef_link 末端执行器链接名称。
 * @param timeout_s IK 请求超时时间（秒）。
 * @param wait_service 是否在发起请求前等待 /compute_ik 服务可用。
 * @param logger ROS 日志器。
 * @return IK 求解结果，包含解向量和错误码；若失败则解为空。
 */
IkResult solve_ik(const rclcpp::Client<GetPositionIK>::SharedPtr& client,
                  const geometry_msgs::msg::PoseStamped& pose_stamped,
                  const std::vector<double>& seed,
                  const std::vector<std::string>& joint_names,
                  const std::string& group, const std::string& eef_link,
                  double timeout_s, bool wait_service, rclcpp::Logger logger) {
    if (wait_service) {
        if (!client->wait_for_service(500ms)) {
            RCLCPP_WARN(logger, "/compute_ik 服务不可用");
            return {std::nullopt, -1};
        }
    } else if (!client->service_is_ready()) {
        // 批量场景：服务可用性已在批量开始检查一次，此处只做廉价本地判断
        return {std::nullopt, -1};
    }

    // 组装 IK 请求：不做碰撞检查（避障由上层的 /check_state_validity 负责），
    // 种子取当前位形以保证解的连续性
    auto req = std::make_shared<GetPositionIK::Request>();
    req->ik_request.group_name = group;
    if (!eef_link.empty()) {
        req->ik_request.ik_link_name = eef_link;
    }
    req->ik_request.pose_stamped = pose_stamped;
    req->ik_request.avoid_collisions = false;
    req->ik_request.timeout.sec = static_cast<int32_t>(timeout_s);
    req->ik_request.timeout.nanosec = static_cast<uint32_t>(
        (timeout_s - static_cast<int32_t>(timeout_s)) * 1e9);
    req->ik_request.robot_state.joint_state.name = joint_names;
    req->ik_request.robot_state.joint_state.position = seed;

    // 阻塞等 future：超时给足服务端超时的 2 倍，留出服务往返与排队余量。
    // 超时必须 remove_pending_request —— 否则该 future 会一直挂在客户端里，
    // 后续请求的回调可能被这条陈旧记录干扰
    auto future = client->async_send_request(req);
    const double wait_s = timeout_s * 2.0;
    if (future.wait_for(std::chrono::duration<double>(wait_s)) !=
        std::future_status::ready) {
        RCLCPP_WARN(logger, "IK 超时 (%.2fs)", wait_s);
        client->remove_pending_request(future);
        return {std::nullopt, -1};
    }

    auto resp = future.get();
    if (!resp ||
        resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        return {std::nullopt, resp ? resp->error_code.val : -1};
    }

    // 按 joint_names 顺序重排（不依赖求解器返回顺序）
    std::map<std::string, double> n2p;
    const auto& js = resp->solution.joint_state;
    const size_t m = std::min(js.name.size(), js.position.size());
    for (size_t i = 0; i < m; ++i) n2p[js.name[i]] = js.position[i];

    std::vector<double> out;
    out.reserve(joint_names.size());
    for (const auto& n : joint_names) {
        auto it = n2p.find(n);
        out.push_back(it != n2p.end() ? it->second : 0.0);
    }
    return {std::move(out),
            static_cast<int>(moveit_msgs::msg::MoveItErrorCodes::SUCCESS)};
}

}  // namespace robot_arm_node::motion
