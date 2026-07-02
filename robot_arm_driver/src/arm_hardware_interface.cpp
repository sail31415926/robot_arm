/**
 * @file arm_hardware_interface.cpp
 * @brief ArmHardwareInterface 实现（见头文件说明）
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 EMEET
 */

#include "robot_arm_driver/arm_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

using namespace std::chrono_literals;
using CallbackReturn = hardware_interface::CallbackReturn;
using return_type    = hardware_interface::return_type;

namespace robot_arm_driver {

// ── 小工具：从字符串 map 取参数 ─────────────────────────────────────────────
namespace {
std::string getParam(const std::unordered_map<std::string, std::string> & m,
                     const std::string & key, const std::string & def)
{
    auto it = m.find(key);
    return (it != m.end()) ? it->second : def;
}
}  // namespace

// ── on_init ─────────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    const auto & hp = info_.hardware_parameters;

    // ── 参数来源：URDF <hardware> 给了 config_file 则从该 yaml 读，否则回退 URDF <param> ──
    const std::string config_file = getParam(hp, "config_file", "");
    YAML::Node hwcfg;
    bool have_cfg = false;
    if (!config_file.empty()) {
        try {
            hwcfg = YAML::LoadFile(config_file)["arm_hardware"];
        } catch (const std::exception & e) {
            RCLCPP_ERROR(logger_, "读取 config_file 失败 (%s): %s", config_file.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
        if (!hwcfg) {
            RCLCPP_ERROR(logger_, "config_file %s 缺少 'arm_hardware' 根键", config_file.c_str());
            return CallbackReturn::ERROR;
        }
        have_cfg = true;
    }

    // 统一取值：优先 yaml（config_file），否则 URDF <param>，都无则默认（均以字符串中转）
    auto pstr = [&](const std::string & key, const std::string & def) -> std::string {
        if (have_cfg && hwcfg[key]) return hwcfg[key].as<std::string>();
        return getParam(hp, key, def);
    };
    try {
        can_if_            = pstr("can_interface",    "can0");
        sdo_timeout_ms_    = std::stoi(pstr("sdo_timeout_ms",    "500"));
        ip_period_ms_      = std::stoi(pstr("ip_period_ms",      "10"));
        feedback_fast_ms_  = std::stoi(pstr("feedback_fast_ms",  "20"));
        enable_stagger_ms_ = std::stoi(pstr("enable_stagger_ms", "150"));
        heartbeat_ms_      = std::stoi(pstr("heartbeat_ms",      "100"));
        pp_accel_          = std::stod(pstr("pp_accel",          "1.0"));
        pp_decel_          = std::stod(pstr("pp_decel",          "1.0"));
        master_node_id_    = static_cast<uint8_t>(std::stoi(pstr("master_node_id", "127")));
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "hardware 参数解析失败: %s", e.what());
        return CallbackReturn::ERROR;
    }

    // sim_mode 始终由 URDF 决定（backend / launch 的 arm_sim_mode 开关，不放 yaml）
    std::string sm = getParam(hp, "sim_mode", "false");   // 容错：True/true/1 均视为真
    std::transform(sm.begin(), sm.end(), sm.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    sim_mode_ = (sm == "true" || sm == "1");

    const size_t n = info_.joints.size();
    pos_.assign(n, 0.0);   vel_.assign(n, 0.0);
    cmd_pos_.assign(n, 0.0); cmd_vel_.assign(n, 0.0);
    fb_pos_.assign(n, 0.0);  fb_vel_.assign(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
        const auto & jp    = info_.joints[i].parameters;
        const std::string & jname = info_.joints[i].name;
        int64_t node_id = 0, cpr = 524288;
        double  mv = 1.0;
        try {
            if (have_cfg) {   // 从 yaml 的 joints[<关节名>] 读
                YAML::Node jc = hwcfg["joints"] ? hwcfg["joints"][jname] : YAML::Node();
                if (!jc) {
                    RCLCPP_ERROR(logger_, "config_file 缺少关节 '%s' 的配置", jname.c_str());
                    return CallbackReturn::ERROR;
                }
                node_id = jc["node_id"].as<int64_t>();
                cpr     = jc["counts_per_rev"] ? jc["counts_per_rev"].as<int64_t>() : 524288;
                mv      = jc["max_velocity"]   ? jc["max_velocity"].as<double>()    : 1.0;
            } else {          // 回退：URDF <joint><param>
                node_id = std::stoll(getParam(jp, "node_id", "0"));
                cpr     = std::stoll(getParam(jp, "counts_per_rev", "524288"));
                mv      = std::stod(getParam(jp, "max_velocity", "1.0"));
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(logger_, "关节 %s 参数解析失败: %s", jname.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
        if (node_id <= 0) {
            RCLCPP_ERROR(logger_, "关节 %s 缺少有效 node_id 参数",
                         info_.joints[i].name.c_str());
            return CallbackReturn::ERROR;
        }

        node_ids_.push_back(node_id);
        counts_per_rev_.push_back(cpr);
        max_vel_.push_back(mv);
        converters_.emplace_back(cpr);
        drivers_.push_back(std::make_unique<arm::CanopenMotorDriver>(
            can_if_, static_cast<uint8_t>(node_id), sdo_timeout_ms_));
        can_mtx_.push_back(std::make_unique<std::mutex>());

        // 校验接口：需 position+velocity 的 command 与 state
        if (info_.joints[i].command_interfaces.size() < 1 ||
            info_.joints[i].state_interfaces.size()   < 1) {
            RCLCPP_ERROR(logger_, "关节 %s 缺少 command/state 接口",
                         info_.joints[i].name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(logger_,
        "on_init OK — %zu 关节  can=%s  ip_period=%dms  fb=%dHz  sim_mode=%s",
        n, can_if_.c_str(), ip_period_ms_,
        feedback_fast_ms_ > 0 ? 1000 / feedback_fast_ms_ : 0,
        sim_mode_ ? "true" : "false");
    return CallbackReturn::SUCCESS;
}

// ── on_configure：打开 SocketCAN ────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    if (sim_mode_) {
        RCLCPP_INFO(logger_, "sim_mode：跳过 CAN 初始化");
        return CallbackReturn::SUCCESS;
    }
    for (size_t i = 0; i < drivers_.size(); ++i) {
        if (!drivers_[i]->init()) {
            RCLCPP_ERROR(logger_, "CAN init 失败  joint=%s  node_id=%ld  if=%s",
                         info_.joints[i].name.c_str(), node_ids_[i], can_if_.c_str());
            return CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(logger_, "SocketCAN 已打开 (%s)", can_if_.c_str());
    return CallbackReturn::SUCCESS;
}

// ── on_activate：使能 + 切 IP 模式 + 播种命令值 + 起反馈线程 ─────────────────
CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    if (sim_mode_) {
        for (size_t i = 0; i < pos_.size(); ++i) {
            cmd_pos_[i] = pos_[i] = 0.0;
            cmd_vel_[i] = vel_[i] = 0.0;
            fb_pos_[i] = fb_vel_[i] = 0.0;
        }
        active_.store(true);
        RCLCPP_INFO(logger_, "Activated (sim_mode)");
        return CallbackReturn::SUCCESS;
    }

    // NMT Start + 残留故障复位（此时反馈线程尚未启动，单线程，无需加锁）
    for (auto & d : drivers_) d->nmtCommand(0x01);
    std::this_thread::sleep_for(50ms);
    for (auto & d : drivers_) {
        if (d->getStatusWord() & 0x0008u) {   // bit3 = Fault
            d->resetFault();
            std::this_thread::sleep_for(20ms);
        }
    }

    if (!setAllMode(Mode::IP)) {
        RCLCPP_WARN(logger_, "部分关节 IP 模式切换失败");
    }
    mode_ = Mode::IP;

    // 播种命令值为当前实测位置，避免激活首周期跳变
    for (size_t i = 0; i < drivers_.size(); ++i) {
        double p = converters_[i].ppToRad(drivers_[i]->getPosition());
        pos_[i] = cmd_pos_[i] = fb_pos_[i] = p;
        vel_[i] = fb_vel_[i] = 0.0;
    }

    startFeedback();
    active_.store(true);
    RCLCPP_INFO(logger_, "Activated (IP mode)");
    return CallbackReturn::SUCCESS;
}

// ── on_deactivate ───────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    active_.store(false);
    stopFeedback();
    if (!sim_mode_) {
        for (size_t i = 0; i < drivers_.size(); ++i) {
            std::lock_guard<std::mutex> lk(*can_mtx_[i]);
            drivers_[i]->disable();
        }
    }
    RCLCPP_INFO(logger_, "Deactivated%s", sim_mode_ ? " (sim_mode)" : "");
    return CallbackReturn::SUCCESS;
}

// ── on_cleanup ──────────────────────────────────────────────────────────────
CallbackReturn ArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
    if (!sim_mode_) {
        for (auto & d : drivers_) d->close();
    }
    return CallbackReturn::SUCCESS;
}

// ── 接口导出 ────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
        si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    }
    return si;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
        ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    }
    return ci;
}

// ── 模式切换 ────────────────────────────────────────────────────────────────
hardware_interface::return_type ArmHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
{
    // 全部接受：JTC 会同时占用 position + velocity（位置指令 + 速度前馈），属正常组合；
    // 仅 velocity 而无 position 才是 PV（速度）控制。实际模式在 perform 阶段判定。
    return return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & /*stop_interfaces*/)
{
    if (sim_mode_ || start_interfaces.empty()) return return_type::OK;

    bool want_vel = false, want_pos = false;
    for (const auto & s : start_interfaces) {
        if (s.find("/velocity") != std::string::npos) want_vel = true;
        if (s.find("/position") != std::string::npos) want_pos = true;
    }

    // 有 position（含 position+velocity 的 JTC）→ IP；仅 velocity → PV
    const Mode target = (want_vel && !want_pos) ? Mode::PV : Mode::IP;
    if (target == mode_) return return_type::OK;

    if (target == Mode::PV) {
        for (size_t i = 0; i < drivers_.size(); ++i) {   // 切换前停零防毛刺
            std::lock_guard<std::mutex> lk(*can_mtx_[i]);
            drivers_[i]->setTargetVelocity(0);
        }
        if (setAllMode(Mode::PV)) { mode_ = Mode::PV; RCLCPP_INFO(logger_, "切换到 PV 模式"); }
    } else {
        if (setAllMode(Mode::IP)) { mode_ = Mode::IP; RCLCPP_INFO(logger_, "切换到 IP 模式"); }
    }
    return return_type::OK;
}

// ── read：拷贝后台反馈缓存 ──────────────────────────────────────────────────
hardware_interface::return_type ArmHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    if (sim_mode_) {   // 回显命令为状态
        for (size_t i = 0; i < pos_.size(); ++i) {
            pos_[i] = cmd_pos_[i];
            vel_[i] = cmd_vel_[i];
        }
        return return_type::OK;
    }
    std::lock_guard<std::mutex> lk(fb_mtx_);
    for (size_t i = 0; i < pos_.size(); ++i) {
        pos_[i] = fb_pos_[i];
        vel_[i] = fb_vel_[i];
    }
    return return_type::OK;
}

// ── write：IP 逐点 + SYNC / PV 速度 + 心跳 ──────────────────────────────────
hardware_interface::return_type ArmHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration & period)
{
    if (sim_mode_ || !active_.load()) return return_type::OK;

    if (mode_ == Mode::IP) {
        for (size_t i = 0; i < drivers_.size(); ++i) {
            int32_t pp = converters_[i].radToPP(cmd_pos_[i]);
            std::lock_guard<std::mutex> lk(*can_mtx_[i]);
            drivers_[i]->sendInterpolationData(pp);
        }
        {   // 一条 SYNC 触发三轴同步执行本周期插补点
            std::lock_guard<std::mutex> lk(*can_mtx_[0]);
            drivers_[0]->sendSYNC();
        }
    } else {  // PV
        for (size_t i = 0; i < drivers_.size(); ++i) {
            int32_t v = converters_[i].radToVelPPSigned(cmd_vel_[i]);
            std::lock_guard<std::mutex> lk(*can_mtx_[i]);
            drivers_[i]->setTargetVelocity(v);
        }
    }

    // 心跳：每 heartbeat_ms 由 node0 驱动发一次
    hb_accum_s_ += period.seconds();
    if (heartbeat_ms_ > 0 && hb_accum_s_ >= heartbeat_ms_ * 1e-3) {
        std::lock_guard<std::mutex> lk(*can_mtx_[0]);
        drivers_[0]->sendHeartbeat(master_node_id_);
        hb_accum_s_ = 0.0;
    }
    return return_type::OK;
}

// ── DS402 模式切换（含错峰上电）─────────────────────────────────────────────
bool ArmHardwareInterface::setAllMode(Mode m)
{
    bool ok = true;
    for (size_t i = 0; i < drivers_.size(); ++i) {
        if (i > 0 && enable_stagger_ms_ > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(enable_stagger_ms_));
        }
        std::lock_guard<std::mutex> lk(*can_mtx_[i]);
        if (m == Mode::IP) {
            uint32_t max_vel_pp = converters_[i].radToVelPP(max_vel_[i]);
            ok = drivers_[i]->setInterpolatedPositionMode(
                     static_cast<uint8_t>(ip_period_ms_), max_vel_pp) && ok;
        } else {  // PV
            uint32_t a = converters_[i].radToAccPP(pp_accel_);
            uint32_t d = converters_[i].radToAccPP(pp_decel_);
            ok = drivers_[i]->enable() && ok;
            ok = drivers_[i]->setProfileVelocityMode(a, d) && ok;
        }
    }
    return ok;
}

// ── 后台反馈线程 ────────────────────────────────────────────────────────────
void ArmHardwareInterface::startFeedback()
{
    fb_running_.store(true);
    fb_thread_ = std::thread(&ArmHardwareInterface::feedbackLoop, this);
}

void ArmHardwareInterface::stopFeedback()
{
    fb_running_.store(false);
    if (fb_thread_.joinable()) fb_thread_.join();
}

void ArmHardwareInterface::feedbackLoop()
{
    const auto dt = std::chrono::milliseconds(feedback_fast_ms_ > 0 ? feedback_fast_ms_ : 20);
    while (fb_running_.load()) {
        for (size_t i = 0; i < drivers_.size(); ++i) {
            double p, v;
            {
                std::lock_guard<std::mutex> lk(*can_mtx_[i]);
                p = converters_[i].ppToRad(drivers_[i]->getPosition());
                v = converters_[i].ppToRadS(drivers_[i]->getVelocity());
            }
            std::lock_guard<std::mutex> lk(fb_mtx_);
            fb_pos_[i] = p;
            fb_vel_[i] = v;
        }
        std::this_thread::sleep_for(dt);
    }
}

}  // namespace robot_arm_driver

PLUGINLIB_EXPORT_CLASS(
    robot_arm_driver::ArmHardwareInterface,
    hardware_interface::SystemInterface)
