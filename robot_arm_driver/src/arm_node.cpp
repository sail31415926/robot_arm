/**
 * @file arm_node.cpp
 * @brief 三关节机械臂整体节点（对外标准 JointTrajectory / joint_states 接口）
 *
 * 将 3 个 CanopenMotorDriver 封装在同一节点内，向上暴露 ROS 2 标准接口：
 *
 *   订阅  /arm_controller/joint_trajectory   trajectory_msgs/JointTrajectory
 *   发布  /joint_states                       sensor_msgs/JointState  (Joint1~3)
 *
 * 运动模式：由启动参数 motion_mode 选择
 *   "ip"（默认）— 插补位置模式，waypoints 间线性插补，PDO + SYNC 同步执行
 *   "pp"        — 轮廓位置模式，取最终 waypoint 为目标，驱动器内部生成速度轮廓
 *
 * 服务：
 *   /arm_node/enable       — 使能所有关节
 *   /arm_node/disable      — 禁用所有关节
 *   /arm_node/recover      — 故障复位并重新使能
 *   /arm_node/set_home     — 将当前位置记为零点
 *
 * 启动方式：
 *   ros2 launch robot_arm_bringup real.launch.py          # 随实物 launch 一键启动
 *   ros2 run robot_arm_driver arm_node \
 *     --ros-args --params-file install/robot_arm_driver/share/robot_arm_driver/config/arm.yaml
 *
 * 参数文件：robot_arm_driver/config/arm.yaml
 *
 * @version 3.0  (dual mode: IP / PP)
 * @date 2026-06-01
 * @copyright Copyright (c) 2026 EMEET
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "canopen_motor_driver/canopen_motor_driver.hpp"
#include "canopen_motor_driver/motor_unit_converter.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using JointState          = sensor_msgs::msg::JointState;
using JointTrajectory     = trajectory_msgs::msg::JointTrajectory;
using FollowJointTraj     = control_msgs::action::FollowJointTrajectory;
using GoalHandleFTJ       = rclcpp_action::ServerGoalHandle<FollowJointTraj>;
using Trigger             = std_srvs::srv::Trigger;

// ─────────────────────────────────────────────────────────────────────────────

class ArmNode : public rclcpp::Node
{
public:
    ArmNode() : Node("arm_node")
    {
        // ── 公共参数 ──────────────────────────────────────────────────────────
        const std::string can_if  = declare_parameter<std::string>("can_interface", "can0");
        const int sdo_timeout_ms  = declare_parameter<int>("sdo_timeout_ms", 500);
        const int hb_ms           = declare_parameter<int>("heartbeat_ms", 100);
        master_node_id_           = static_cast<uint8_t>(
                                        declare_parameter<int>("master_node_id", 127));
        const int fb_fast_ms      = declare_parameter<int>("feedback_fast_ms", 20);
        ip_period_ms_             = declare_parameter<int>("ip_period_ms", 10);
        pp_accel_                 = declare_parameter<double>("pp_accel", 5.0);
        pp_decel_                 = declare_parameter<double>("pp_decel", 5.0);
        motion_mode_              = declare_parameter<std::string>("motion_mode", "ip");
        // 逐个关节使能之间的延时（ms）：错峰上电，降低三电机同时通电的瞬时涌流，
        // 避免共用供电/USB 的摄像头因电压跌落而掉线。设为 0 即恢复同时使能。
        enable_stagger_ms_        = declare_parameter<int>("enable_stagger_ms", 150);
        const bool auto_enable    = declare_parameter<bool>("auto_enable", true);

        // ── 各关节参数（等长数组） ────────────────────────────────────────────
        joint_names_   = declare_parameter<std::vector<std::string>>(
                             "joint_names",   {"Joint1","Joint2","Joint3"});
        auto node_ids  = declare_parameter<std::vector<int64_t>>("node_ids", {1,2,3});
        auto cprs      = declare_parameter<std::vector<int64_t>>(
                             "counts_per_rev", {524288,524288,524288});
        max_vel_       = declare_parameter<std::vector<double>>(
                             "max_velocities",     {3.14, 3.14, 3.14});
        rated_torques_ = declare_parameter<std::vector<double>>(
                             "rated_torques",      {5.0, 20.0, 5.0});

        n_ = joint_names_.size();

        // ── 初始化驱动器 ──────────────────────────────────────────────────────
        home_offsets_.assign(n_, 0);

        for (size_t i = 0; i < n_; ++i) {
            auto drv = std::make_unique<arm::CanopenMotorDriver>(
                can_if, static_cast<uint8_t>(node_ids[i]), sdo_timeout_ms);

            if (!drv->init()) {
                RCLCPP_FATAL(get_logger(),
                    "CAN init 失败  joint=%s  node_id=%ld  if=%s",
                    joint_names_[i].c_str(), node_ids[i], can_if.c_str());
                throw std::runtime_error("CAN init failed");
            }
            drivers_.push_back(std::move(drv));
            converters_.emplace_back(cprs[i]);
        }
        RCLCPP_INFO(get_logger(), "CAN 驱动初始化完成，%zu 个关节", n_);

        // ── 自动使能 & 切换运动模式 ───────────────────────────────────────────
        // IP 模式：setInterpolatedPositionMode() 内含完整 DS402 状态机，不单独调 enable()
        // PP 模式：需先调 enable() 再调 setProfilePositionMode()
        if (auto_enable) {
            for (size_t i = 0; i < n_; ++i) {
                if (!drivers_[i]->nmtCommand(0x01)) {
                    RCLCPP_WARN(get_logger(), "关节 %s NMT Start 失败",
                        joint_names_[i].c_str());
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            for (size_t i = 0; i < n_; ++i) {
                // 上电后若有残留故障，复位一次
                uint16_t sw = drivers_[i]->getStatusWord();
                if (sw & 0x0008u) {
                    drivers_[i]->resetFault();
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }

            if (motion_mode_ == "pp") {
                if (!setAllPPMode())
                    RCLCPP_WARN(get_logger(), "部分关节 PP 模式切换失败");
            } else {
                if (!setAllIPMode())
                    RCLCPP_WARN(get_logger(), "部分关节 IP 模式切换失败");
            }
        }

        // ── 发布 / 订阅 ───────────────────────────────────────────────────────
        js_pub_ = create_publisher<JointState>("/joint_states", 10);

        // 云台轨迹转发：Joint4/5/6 → /gimbal_controller/joint_trajectory
        camera_traj_pub_ = create_publisher<JointTrajectory>(
            "/gimbal_controller/joint_trajectory", 10);

        traj_sub_ = create_subscription<JointTrajectory>(
            "/arm_controller/joint_trajectory", 10,
            std::bind(&ArmNode::onTrajectory, this, std::placeholders::_1));

        // ── FollowJointTrajectory action server（MoveIt 直接调用）────────────
        action_server_ = rclcpp_action::create_server<FollowJointTraj>(
            this, "/arm_controller/follow_joint_trajectory",
            [this](const rclcpp_action::GoalUUID&,
                   std::shared_ptr<const FollowJointTraj::Goal> goal)
                { return handleActionGoal(goal); },
            [this](std::shared_ptr<GoalHandleFTJ> gh)
                { return handleActionCancel(gh); },
            [this](std::shared_ptr<GoalHandleFTJ> gh)
                { handleActionAccepted(gh); });

        // ── 服务 ─────────────────────────────────────────────────────────────
        srv_enable_ = create_service<Trigger>("~/enable",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                stopExecution();
                abortActiveGoal("motor disabled");
                bool ok = true;
                for (auto& d : drivers_) ok = d->nmtCommand(0x01) && ok;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                if (ok) ok = (motion_mode_ == "pp") ? setAllPPMode() : setAllIPMode();
                res->success = ok;
                res->message = ok ? "所有关节已使能" : "部分关节使能失败";
            });

        srv_disable_ = create_service<Trigger>("~/disable",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                stopExecution();
                abortActiveGoal("motor disabled");
                bool ok = true;
                for (auto& d : drivers_) ok = d->disable() && ok;
                res->success = ok;
                res->message = ok ? "所有关节已禁用" : "部分关节禁用失败";
            });

        srv_recover_ = create_service<Trigger>("~/recover",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                stopExecution();
                abortActiveGoal("motor recovery");
                bool ok = true;
                for (auto& d : drivers_) {
                    d->resetFault();
                    rclcpp::sleep_for(200ms);
                    ok = d->nmtCommand(0x01) && ok;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                if (ok) ok = (motion_mode_ == "pp") ? setAllPPMode() : setAllIPMode();
                res->success = ok;
                res->message = ok ? "故障复位并重新使能成功" : "复位后使能失败";
            });

        srv_set_home_ = create_service<Trigger>("~/set_home",
            [this](Trigger::Request::ConstSharedPtr, Trigger::Response::SharedPtr res) {
                for (size_t i = 0; i < n_; ++i)
                    home_offsets_[i] = drivers_[i]->getPosition();
                res->success = true;
                res->message = "当前位置已记为各关节零点";
            });

        // ── 反馈定时器 ────────────────────────────────────────────────────────
        fb_fast_timer_ = create_wall_timer(
            std::chrono::milliseconds(fb_fast_ms),
            [this]() { publishFeedback(); });

        // ── 心跳（joint1 驱动负责）──────────────────────────────────────────
        if (hb_ms > 0 && !drivers_.empty()) {
            hb_timer_ = create_wall_timer(
                std::chrono::milliseconds(hb_ms),
                [this]() { drivers_[0]->sendHeartbeat(master_node_id_); });
        }

        RCLCPP_INFO(get_logger(),
            "arm_node 启动完成  (mode=%s)\n"
            "  Action  /arm_controller/follow_joint_trajectory\n"
            "  Topic   /arm_controller/joint_trajectory\n"
            "  Publish /joint_states\n"
            "  Service ~/enable  ~/disable  ~/recover  ~/set_home",
            motion_mode_.c_str());
    }

private:
    // ── IP 模式初始化 ─────────────────────────────────────────────────────────

    bool setAllIPMode()
    {
        bool ok = true;
        for (size_t i = 0; i < n_; ++i) {
            uint32_t max_vel_pp = converters_[i].radToVelPP(max_vel_[i]);
            // setInterpolatedPositionMode 内部会接通功率级（controlword 0x000F），
            // 逐个上电之间错峰，避免三电机同时通电的瞬时涌流
            if (i > 0 && enable_stagger_ms_ > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(enable_stagger_ms_));
            ok = drivers_[i]->setInterpolatedPositionMode(
                     static_cast<uint8_t>(ip_period_ms_), max_vel_pp) && ok;
        }
        return ok;
    }

    bool setAllPPMode()
    {
        bool ok = true;
        for (size_t i = 0; i < n_; ++i) {
            // enable() 接通功率级（controlword 0x000F）；逐个上电之间错峰，
            // 避免三电机同时通电的瞬时涌流（见 enable_stagger_ms 参数）
            if (i > 0 && enable_stagger_ms_ > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(enable_stagger_ms_));
            ok = drivers_[i]->enable() && ok;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (size_t i = 0; i < n_; ++i) {
            uint32_t v = converters_[i].radToVelPP(max_vel_[i]);
            uint32_t a = converters_[i].radToAccPP(pp_accel_);
            uint32_t d = converters_[i].radToAccPP(pp_decel_);
            ok = drivers_[i]->setProfilePositionMode(v, a, d) && ok;
        }
        return ok;
    }

    void stopExecution()
    {
        if (ip_timer_)   { ip_timer_->cancel();   ip_timer_.reset(); }
        if (done_timer_) { done_timer_->cancel(); done_timer_.reset(); }
        pending_ = JointTrajectory{};
        ip_elapsed_ = 0.0;
        ip_total_dur_ = 0.0;
        ip_seg_idx_ = 0;
    }

    void abortActiveGoal(const std::string& reason)
    {
        if (!active_goal_) return;
        auto res = std::make_shared<FollowJointTraj::Result>();
        res->error_code = FollowJointTraj::Result::INVALID_GOAL;
        res->error_string = reason;
        active_goal_->abort(res);
        active_goal_.reset();
    }

    void finishActiveGoal()
    {
        if (!active_goal_) return;
        auto res = std::make_shared<FollowJointTraj::Result>();
        res->error_code = FollowJointTraj::Result::SUCCESSFUL;
        active_goal_->succeed(res);
        active_goal_.reset();
    }

    // ── FollowJointTrajectory action server ─────────────────────────────────

    rclcpp_action::GoalResponse handleActionGoal(
        std::shared_ptr<const FollowJointTraj::Goal> goal)
    {
        for (const auto& name : goal->trajectory.joint_names) {
            for (const auto& jn : joint_names_) {
                if (name == jn) return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
        }
        RCLCPP_WARN(get_logger(), "rejected: trajectory 不含 Joint1-3");
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse handleActionCancel(
        std::shared_ptr<GoalHandleFTJ> /*gh*/)
    {
        stopExecution();
        abortActiveGoal("cancelled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleActionAccepted(std::shared_ptr<GoalHandleFTJ> gh)
    {
        auto traj = std::make_shared<JointTrajectory>(gh->get_goal()->trajectory);
        onTrajectory(traj);  // 内部 abort 旧 goal + stopExecution
        active_goal_ = gh;   // onTrajectory 完成后才设置，避免被自己 abort
    }

    // ── 轨迹回调 ────────────────────────────────────────────────────────────

    void onTrajectory(const JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty() || msg->joint_names.empty()) return;

        // 抢占：停止当前执行并 abort 旧 action goal
        abortActiveGoal("preempted by new trajectory");
        stopExecution();

        // ── 分流：提取 Joint4/5/6 → gimbal_controller ────────────────────────
        static const std::set<std::string> CAM_JOINTS{"Joint4","Joint5","Joint6"};
        std::vector<size_t> cam_idx;
        std::vector<std::string> cam_names;
        for (size_t k = 0; k < msg->joint_names.size(); ++k) {
            if (CAM_JOINTS.count(msg->joint_names[k])) {
                cam_names.push_back(msg->joint_names[k]);
                cam_idx.push_back(k);
            }
        }
        if (!cam_names.empty()) {
            JointTrajectory cam_traj;
            cam_traj.header.stamp    = this->now();
            cam_traj.header.frame_id = msg->header.frame_id;
            cam_traj.joint_names = cam_names;
            for (const auto& pt : msg->points) {
                trajectory_msgs::msg::JointTrajectoryPoint np;
                np.time_from_start = pt.time_from_start;
                for (size_t k : cam_idx) {
                    if (k < pt.positions.size())  np.positions.push_back(pt.positions[k]);
                    if (k < pt.velocities.size()) np.velocities.push_back(pt.velocities[k]);
                }
                cam_traj.points.push_back(np);
            }
            camera_traj_pub_->publish(cam_traj);
        }

        // ── Joint1-3：按 motion_mode_ 选择执行方式 ──────────────────────────
        stopExecution();
        pending_ = *msg;

        // 建立 joint_name → 驱动索引的映射
        arm_joint_map_.clear();
        for (size_t j = 0; j < n_; ++j) {
            for (size_t k = 0; k < pending_.joint_names.size(); ++k) {
                if (pending_.joint_names[k] == joint_names_[j]) {
                    arm_joint_map_[j] = k;
                    break;
                }
            }
        }

        if (motion_mode_ == "pp") {
            // PP 模式：取最终 waypoint，按距离/时间动态设轮廓速度，非阻塞触发
            const auto& last_pt = pending_.points.back();
            double total_dur = last_pt.time_from_start.sec +
                               last_pt.time_from_start.nanosec * 1e-9;

            for (size_t j = 0; j < n_; ++j) {
                auto it = arm_joint_map_.find(j);
                if (it == arm_joint_map_.end()) continue;
                size_t k = it->second;
                if (k >= last_pt.positions.size()) continue;

                int32_t target_pp = converters_[j].radToPP(last_pt.positions[k])
                                   + home_offsets_[j];

                if (total_dur > 0.01) {
                    int32_t cur_pp  = drivers_[j]->getPosition();
                    double dist_rad = std::abs(converters_[j].ppToRad(target_pp - cur_pp));
                    double vel_rad  = std::clamp(dist_rad / total_dur, 0.001, max_vel_[j]);
                    uint32_t vel_pp = converters_[j].radToVelPP(vel_rad);
                    if (vel_pp > 0) drivers_[j]->setProfileVelocity(vel_pp);
                }

                drivers_[j]->moveToPosition(target_pp, false, false);
            }

            // 每 20ms 轮询状态字 bit10（Target Reached），到位后通知 action goal
            done_timer_ = create_wall_timer(20ms, [this]() { checkDone(); });
        } else {
            // IP 模式：线性插补 + PDO + SYNC 周期定时器
            const auto& last_pt = pending_.points.back();
            ip_total_dur_ = last_pt.time_from_start.sec +
                            last_pt.time_from_start.nanosec * 1e-9;
            ip_elapsed_ = 0.0;
            ip_seg_idx_ = 0;

            interpolateAndSend();

            if (ip_total_dur_ > 0.0) {
                ip_timer_ = create_wall_timer(
                    std::chrono::milliseconds(ip_period_ms_),
                    [this]() { interpolateAndSend(); });
            }
        }
    }

    void interpolateAndSend()
    {
        if (pending_.points.empty()) {
            stopExecution();
            return;
        }

        // ── 线性插补：根据 ip_elapsed_ 在 waypoints 间计算各关节位置 ────────
        const auto& pts = pending_.points;

        // 找到当前段：pts[seg] 到 pts[seg+1]，使得 seg.time <= elapsed < seg+1.time
        while (ip_seg_idx_ + 1 < pts.size()) {
            double t_next = pts[ip_seg_idx_ + 1].time_from_start.sec +
                            pts[ip_seg_idx_ + 1].time_from_start.nanosec * 1e-9;
            if (ip_elapsed_ < t_next) break;
            ++ip_seg_idx_;
        }

        // 边界：已经到最后一个 waypoint
        if (ip_seg_idx_ + 1 >= pts.size()) {
            // 保持最后一个 waypoint 位置发最后一帧，然后结束
            const auto& pt = pts.back();
            for (size_t j = 0; j < n_; ++j) {
                auto it = arm_joint_map_.find(j);
                if (it != arm_joint_map_.end() && it->second < pt.positions.size()) {
                    int32_t pp = converters_[j].radToPP(pt.positions[it->second])
                               + home_offsets_[j];
                    drivers_[j]->sendInterpolationData(pp);
                }
            }
            sendSYNC();
            stopExecution();
            finishActiveGoal();
            return;
        }

        // 段内线性插补
        const auto& p0 = pts[ip_seg_idx_];
        const auto& p1 = pts[ip_seg_idx_ + 1];
        double t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1e-9;
        double t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9;
        double seg_dur = t1 - t0;
        double alpha = (seg_dur > 1e-9) ? (ip_elapsed_ - t0) / seg_dur : 0.0;
        alpha = std::clamp(alpha, 0.0, 1.0);

        // 每个关节：发 RPDO1 数据（不含 SYNC）
        for (size_t j = 0; j < n_; ++j) {
            auto it = arm_joint_map_.find(j);
            if (it == arm_joint_map_.end()) continue;
            size_t k = it->second;

            double pos0 = (k < p0.positions.size()) ? p0.positions[k] : 0.0;
            double pos1 = (k < p1.positions.size()) ? p1.positions[k] : pos0;
            double pos_rad = pos0 + alpha * (pos1 - pos0);

            int32_t pp = converters_[j].radToPP(pos_rad) + home_offsets_[j];
            drivers_[j]->sendInterpolationData(pp);
        }

        // 所有 RPDO 发完后，一条 SYNC 触发三轴同步执行
        sendSYNC();

        // 推进时间
        ip_elapsed_ += ip_period_ms_ * 1e-3;

        if (ip_elapsed_ >= ip_total_dur_) {
            stopExecution();
            finishActiveGoal();
        }
    }

    /** @brief PP 模式完成监测：轮询 arm_joint_map_ 内各关节状态字 bit10（Target Reached） */
    void checkDone()
    {
        for (const auto& [j, k] : arm_joint_map_) {
            if (!(drivers_[j]->getStatusWord() & 0x0400u)) return;
        }
        done_timer_->cancel();
        done_timer_.reset();
        finishActiveGoal();
    }

    /** @brief 通过任一已打开的驱动器发送 SYNC 广播帧 */
    void sendSYNC() {
        if (!drivers_.empty()) drivers_[0]->sendSYNC();
    }

    // ── 反馈发布 ────────────────────────────────────────────────────────────

    void publishFeedback()
    {
        JointState js;
        js.header.stamp = now();
        js.name.resize(n_);
        js.position.resize(n_);
        js.velocity.resize(n_, 0.0);
        js.effort.resize(n_, 0.0);

        for (size_t i = 0; i < n_; ++i) {
            js.name[i]     = joint_names_[i];
            js.position[i] = converters_[i].ppToRad(
                drivers_[i]->getPosition() - home_offsets_[i]);
        }
        js_pub_->publish(js);
    }

    // ── 成员变量 ─────────────────────────────────────────────────────────────

    size_t n_{3};
    int    ip_period_ms_{10};
    int    enable_stagger_ms_{150};
    double pp_accel_{5.0};
    double pp_decel_{5.0};
    std::string motion_mode_{"ip"};
    std::vector<std::string> joint_names_;
    std::vector<std::unique_ptr<arm::CanopenMotorDriver>> drivers_;
    std::vector<arm::MotorUnitConverter> converters_;
    std::vector<int32_t> home_offsets_;
    std::vector<double>  max_vel_, rated_torques_;
    uint8_t master_node_id_{127};

    rclcpp::Publisher<JointState>::SharedPtr               js_pub_;
    rclcpp::Publisher<JointTrajectory>::SharedPtr          camera_traj_pub_;
    rclcpp::Subscription<JointTrajectory>::SharedPtr       traj_sub_;
    rclcpp::Service<Trigger>::SharedPtr  srv_enable_, srv_disable_,
                                         srv_recover_, srv_set_home_;
    rclcpp_action::Server<FollowJointTraj>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFTJ> active_goal_;
    rclcpp::TimerBase::SharedPtr  fb_fast_timer_, hb_timer_, ip_timer_, done_timer_;

    // 执行状态（IP 插补 / PP 目标发送共用）
    JointTrajectory pending_;
    std::map<size_t, size_t> arm_joint_map_;  ///< 驱动索引 → trajectory.joint_names 索引
    double ip_elapsed_{0.0};
    double ip_total_dur_{0.0};
    size_t ip_seg_idx_{0};
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ArmNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
