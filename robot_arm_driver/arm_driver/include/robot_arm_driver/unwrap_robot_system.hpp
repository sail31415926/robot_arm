/**
 * @file unwrap_robot_system.hpp
 * @brief UnwrapRobotSystem —— 带单圈绝对编码器 2π 折算的 RobotSystem
 *
 * 背景：RB200-CA 输出轴为 19bit 单圈绝对编码器，上电时 6064 只能给出一圈内的
 * 绝对位置（落在 [0, 2π)，恒非负）。关节物理上停在零点负侧 −θ 时反馈读成
 * +2π−θ，直接喂给 JTC 后任何"走到 0"的轨迹都会朝负方向扫过近一整圈、顶到
 * 物理限位（回零永远同一个方向的根因，2026-07 实机复现）。
 *
 * 方案：继承 canopen_ros2_control::RobotSystem，在 read/write 做**对称**折算：
 *   read : report = raw − 2π·k，k = round((raw − center)/2π)；center 取该关节
 *          URDF position 命令接口 (min+max)/2。各关节行程 < 2π ⇒ k 唯一，
 *          折算边界 center±π 落在物理不可达区（硬限位之外），不会在运动中越界。
 *   write: cmd_raw = cmd + 2π·k（用本周期 read 的 k），命令回到驱动器自身
 *          坐标系 —— 只折算读侧会让 JTC 起点与驱动器坐标差 2π 直接超差。
 * 位置本就在窗内时 k=0，行为与原 RobotSystem 完全一致。
 *
 * 所属：robot_arm_driver（自研定制，不改 vendored ros2_canopen，见 VENDOR.md 约定）
 *
 * @version 1.0
 * @date 2026-07-16
 * @copyright Copyright (c) 2026 EMEET
 */
#pragma once

#include <atomic>
#include <vector>

#include "canopen_ros2_control/robot_system.hpp"

namespace robot_arm_driver
{

class UnwrapRobotSystem : public canopen_ros2_control::RobotSystem
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// 换模时把位置目标播种为当前实测位置，避免陈旧命令被回放（无冲击换模）
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  /// 关停前先让电机失力矩（Ctrl-C / kill 时自动失能）。
  ///
  /// 上游 RobotSystem 只在 on_deactivate 里 halt_motor()；on_shutdown / on_cleanup
  /// **只调 clean()**（拆 CANopen master），不失能。而实机上 Ctrl-C 时这两个生命周期
  /// 回调**根本不会被调到** —— 实测 ros2_control_node 关停时以 SIGABRT 挂掉
  /// （exit code -6，vendored 0.2.13 的 DeviceContainer 析构 bug，见 README 已知问题），
  /// 进程在走到组件 shutdown 之前就死了。于是驱动器停在 Operation Enabled 带着力矩：
  /// 机械臂"硬"着不放，且 master 没干净关闭会让驱动器/PCAN 卡在坏状态
  /// （下次 launch 报 SDO timeout、can0 变 NO-CARRIER）。
  ///
  /// 也试过挂 rclcpp::on_shutdown（本文件仍保留），指望 SIGINT 关停上下文时它能早于
  /// 析构执行 —— **实机实测同样不生效**（2026-08-12）：抓 RPDO1 共 11631 帧，控制字
  /// 一直是 0x001F（Operation Enabled）直到总线静默，全程没有任何 halt 报文。
  /// 结论：这条 SIGABRT 路径上**任何进程内钩子都拿不到执行机会**，本文件这几个重写
  /// 只对「能正常走完 deactivate/shutdown」的路径有效（例如显式调 /arm_node/disable）。
  ///
  /// 关停失能因此只能走进程外手段，**2026-08-21 已有工具**：`src/tools/disable_motors.cpp`
  /// （独立 SocketCAN：Quick Stop 0x0002 按 6085 斜坡减速 → Shutdown 0x0006 → 读状态字
  /// 确认 → 失败则 NMT Reset Node `000#8100` 兜底）。但 `real.launch.py` /
  /// `test_arm.launch.py` 的关停钩子**默认不挂**（`auto_disable_on_shutdown:=false`）——
  /// 产品决定：默认保持「Ctrl-C 后臂停在原地、保持力矩、不下沉」，代价是驱动器带电
  /// 且无人控制（`1016` 消费者心跳默认禁用，不会自我保护，一直持续到断电）。
  /// 取舍详见 real.launch.py 钩子处的注释。手动失能不受该开关影响。
  /// 下面这几个重写保留 —— 正常 deactivate 路径下它们更早生效，且不依赖 launch
  /// 或那个开关（例如直接调 controller_manager 服务停组件时）。
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  /// on_shutdown / on_cleanup 共用：仅当组件仍 active 时才需要补 halt
  hardware_interface::CallbackReturn haltIfActive(const char * hook);

  /// 组件是否处于 active（on_activate/on_deactivate 维护）。
  /// on_shutdown 回调在 rclcpp 关停线程里跑，与实时循环并发，故用 atomic。
  std::atomic<bool> active_{false};
  /// rclcpp::on_shutdown 只注册一次
  bool shutdown_hook_registered_{false};

  /// 每关节折算状态，与 robot_motor_data_ 一一对应
  struct Unwrap
  {
    double center = 0.0;  ///< 折算窗中心 = (min+max)/2，rad
    double offset = 0.0;  ///< 2π·k，read 更新、write 加回
    long   k      = 0;    ///< 当前圈偏移（变化时打日志用）
  };
  std::vector<Unwrap> unwrap_;
  std::vector<double> saved_targets_;  ///< write 期间暂存原命令（避免偏移累加）
};

}  // namespace robot_arm_driver
