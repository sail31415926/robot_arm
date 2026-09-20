/**
 * @file motor_debug_backend.cpp
 * @brief 单电机调试平台——总线后端节点（独立 SocketCAN + SDO，一实例一电机）
 *
 * 与 scripts/motor_debug_gui.py（PyQt5 前端）配对，由 launch/motor_debug.launch.py
 * 一起拉起。分工：本节点独占总线（SdoClient + 主站心跳 + 402 状态机 + 单位换算权威），
 * GUI 只经 ROS 话题/服务交互、不碰 CAN。接口全部复用 canopen_interfaces，无自定义消息。
 *
 * 功能：402 使能/失能/故障复位/NMT 复位、PP/PV/PT 三模式点动（rad 系输入）、
 * 编码器置零（HM 方法 35，同 set_encoder_zero 流程）、SDO 裸读写（COReadID/COWriteID
 * 的 canopen_datatype 定长度，nodeid 字段忽略——本节点绑定单电机）、EEPROM 固化、
 * 100ms 轮询反馈（位置/速度/转矩/状态字/模式/故障码）。
 *
 * 话题（~/ = 节点名下）：
 *   ~/state        sensor_msgs/JointState   位置 rad / 速度 rad/s / effort=转矩 %额定
 *   ~/drive_status std_msgs/UInt16MultiArray [状态字, 模式(6061), 故障码(603F)]
 *   ~/log          std_msgs/String          操作结果日志（GUI 日志面板直接显示）
 * 服务：~/{enable,disable,fault_reset,nmt_reset,halt,estop,zero_target,
 *         zero_calibrate,save_eeprom}(Trigger)
 *       ~/{move_pp,run_pv,run_pt}(COTargetDouble)  ~/sdo_read(COReadID) ~/sdo_write(COWriteID)
 * 参数：can_interface / node_id / master_id / sdo_timeout_ms（启动时定）；
 *       pp_profile_velocity / pp_profile_accel（rad/s、rad/s²，GUI 可改）；
 *       counts_per_rad（只读，换算权威 motor_unit_converter.hpp 算出，GUI 取用）
 *
 * ⚠️ 独占总线控制字：运行前必须停掉 ros2_control 栈。
 * ⚠️ 力矩模式下"0"不是"停"是自由下垂；失能会让重力关节下沉。
 * 退出路径（Ctrl-C / launch 关停）：spin 返回后无条件清零 PV/PT 目标 + Shutdown 失能。
 * 单线程执行器 = 所有服务/定时器串行访问 SdoClient，无并发问题（置零等长流程会
 * 阻塞轮询几秒，可接受）。
 *
 * @date 2026-08-17
 * @copyright Copyright (c) 2026 EMEET
 */

#include "robot_arm_driver/sdo_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <canopen_interfaces/srv/co_read_id.hpp>
#include <canopen_interfaces/srv/co_target_double.hpp>
#include <canopen_interfaces/srv/co_write_id.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

using arm_tools::SdoClient;
using arm_tools::rb200;
using std_srvs::srv::Trigger;
using canopen_interfaces::srv::COReadID;
using canopen_interfaces::srv::COTargetDouble;
using canopen_interfaces::srv::COWriteID;
using namespace std::chrono_literals;

/** COReadID/COWriteID 的 canopen_datatype → SDO 字节数。
 *  兼容两种口径：srv 常量（0x02/0x05=8位 0x03/0x06=16位 0x04/0x07=32位）
 *  与注释口径（8/16/32 直接写位宽）。其余值按 4 字节。 */
static int datatypeToSize(uint8_t dt)
{
    switch (dt) {
        case 0x02: case 0x05: case 8:  return 1;
        case 0x03: case 0x06: case 16: return 2;
        default:                       return 4;
    }
}

class MotorDebugBackend : public rclcpp::Node {
public:
    MotorDebugBackend() : Node("motor_debug_backend")
    {
        const auto ifname  = declare_parameter<std::string>("can_interface", "can0");
        const int node_id  = static_cast<int>(declare_parameter<int>("node_id", 1));
        const int master   = static_cast<int>(declare_parameter<int>("master_id", 127));
        const int timeout  = static_cast<int>(declare_parameter<int>("sdo_timeout_ms", 500));
        declare_parameter<double>("pp_profile_velocity", 0.3);   // rad/s，GUI 可改
        declare_parameter<double>("pp_profile_accel", 1.0);      // rad/s²，GUI 可改
        {   // 换算系数只读下发（权威 = motor_unit_converter.hpp，GUI 不得自带常数）
            rcl_interfaces::msg::ParameterDescriptor ro;
            ro.read_only = true;
            declare_parameter<double>("counts_per_rad", 1.0 / rb200().ppToRad(1), ro);
        }
        if (node_id < 1 || node_id > 127)
            throw std::runtime_error("node_id 必须在 1~127");

        // 连不上口/节点无响应直接抛，main 里报错退出（launch 终端可见）
        bus_ = std::make_unique<SdoClient>(ifname, static_cast<uint8_t>(node_id),
                                           static_cast<uint8_t>(master), timeout);
        const uint16_t sw = bus_->statusWord();
        RCLCPP_INFO(get_logger(), "已连接 %s 节点 %d（状态字 0x%04X）",
                    ifname.c_str(), node_id, sw);

        {   // 额定电流（6075，mA）/额定力矩（6076，mNm）→ GUI 算负载率与实际 N·m;
            // 对象缺失/Abort 只降级显示、不拦启动
            double rated_a = 0.0, rated_nm = 0.0;
            try {
                rated_a = bus_->sdoRead(0x6075u, 0) / 1000.0;
            } catch (const std::exception & e) {
                RCLCPP_WARN(get_logger(), "读额定电流 6075 失败（%s），负载率显示不可用", e.what());
            }
            try {
                rated_nm = bus_->sdoRead(0x6076u, 0) / 1000.0;
            } catch (const std::exception & e) {
                RCLCPP_WARN(get_logger(), "读额定力矩 6076 失败（%s），N·m 显示不可用", e.what());
            }
            rcl_interfaces::msg::ParameterDescriptor ro;
            ro.read_only = true;
            declare_parameter<double>("rated_current_a", rated_a, ro);
            declare_parameter<double>("rated_torque_nm", rated_nm, ro);
        }

        state_pub_  = create_publisher<sensor_msgs::msg::JointState>("~/state", 10);
        status_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("~/drive_status", 10);
        log_pub_    = create_publisher<std_msgs::msg::String>("~/log", 10);
        joint_name_ = "node_" + std::to_string(node_id);

        makeTrigger("enable",         [this] { servoEnable(); return "已使能（Operation enabled）"; });
        makeTrigger("disable",        [this] { clearTargets(); shutdown402();
                                               return "已失能（Shutdown，力矩已断）"; });
        makeTrigger("fault_reset",    [this] { faultReset(); return "故障复位完成"; });
        makeTrigger("nmt_reset",      [this] { bus_->nmt(0x81u);
                                               return "已发 NMT Reset Node（驱动器重启，1~2s 后恢复通信）"; });
        makeTrigger("halt",           [this] { bus_->sdoWrite(0x6040u, 0, 0x010Fu, 2);
                                               return "Halt（沿 6084 减速停住，保持使能）"; });
        makeTrigger("estop",          [this] { clearTargets(); shutdown402();
                                               return "急停：目标清零 + Shutdown（重力关节会下沉）"; });
        makeTrigger("zero_target",    [this] { clearTargets(); return "60FF/6071 目标已清零"; });
        makeTrigger("zero_calibrate", [this] { return zeroCalibrate(); });
        makeTrigger("save_eeprom",    [this] { bus_->saveToEeprom();
                                               return "已固化 EEPROM（1010:01h \"save\"）"; });

        move_pp_srv_ = create_service<COTargetDouble>("~/move_pp",
            [this](COTargetDouble::Request::ConstSharedPtr rq, COTargetDouble::Response::SharedPtr rs)
            { rs->success = guard("PP 运动", [&] { movePP(rq->target); }); });
        run_pv_srv_ = create_service<COTargetDouble>("~/run_pv",
            [this](COTargetDouble::Request::ConstSharedPtr rq, COTargetDouble::Response::SharedPtr rs)
            { rs->success = guard("PV 运动", [&] { runPV(rq->target); }); });
        run_pt_srv_ = create_service<COTargetDouble>("~/run_pt",
            [this](COTargetDouble::Request::ConstSharedPtr rq, COTargetDouble::Response::SharedPtr rs)
            { rs->success = guard("PT 运动", [&] { runPT(rq->target); }); });

        sdo_read_srv_ = create_service<COReadID>("~/sdo_read",
            [this](COReadID::Request::ConstSharedPtr rq, COReadID::Response::SharedPtr rs) {
                rs->success = guard("SDO 读", [&] {
                    rs->data = bus_->sdoRead(rq->index, rq->subindex);
                    logMsg(fmt("[ OK ] 读 %04X:%02X = %d (0x%08X)", rq->index, rq->subindex,
                               static_cast<int32_t>(rs->data), rs->data));
                });
            });
        sdo_write_srv_ = create_service<COWriteID>("~/sdo_write",
            [this](COWriteID::Request::ConstSharedPtr rq, COWriteID::Response::SharedPtr rs) {
                rs->success = guard("SDO 写", [&] {
                    const int size = datatypeToSize(rq->canopen_datatype);
                    bus_->sdoWrite(rq->index, rq->subindex, rq->data, size);
                    logMsg(fmt("[ OK ] 写 %04X:%02X ← %d (0x%08X, %d 字节)", rq->index,
                               rq->subindex, static_cast<int32_t>(rq->data), rq->data, size));
                });
            });

        poll_timer_ = create_wall_timer(100ms, [this] { poll(); });
    }

    /** 退出兜底：清零目标 + Shutdown 掉力矩（spin 返回后由 main 调，析构再兜一次） */
    void safeShutdown()
    {
        if (!bus_) return;
        try {
            clearTargets();
            shutdown402();
            RCLCPP_INFO(get_logger(), "退出：目标已清零并失能");
        } catch (...) { /* 总线已死也要让进程退出 */ }
        bus_.reset();
    }

    ~MotorDebugBackend() override { safeShutdown(); }

private:
    // ── 402 基本操作 ──────────────────────────────────────────────────────────
    void shutdown402() { bus_->sdoWrite(0x6040u, 0, 0x0006u, 2); }

    void clearTargets()   // 任何失能/急停/退出路径前先做，防止重新使能瞬间飞车
    {
        bus_->sdoWrite(0x60FFu, 0, 0, 4);
        bus_->sdoWrite(0x6071u, 0, 0, 2);
    }

    void faultResetIfNeeded()
    {
        if (!(bus_->statusWord() & 0x0008u)) return;
        bus_->sdoWrite(0x6040u, 0, 0x0080u, 2);
        std::this_thread::sleep_for(200ms);
        bus_->sdoWrite(0x6040u, 0, 0x0000u, 2);
        std::this_thread::sleep_for(200ms);
    }

    void faultReset()
    {
        if (!(bus_->statusWord() & 0x0008u)) { logMsg("当前无故障"); return; }
        faultResetIfNeeded();
        if (bus_->statusWord() & 0x0008u)
            throw std::runtime_error(fmt("复位后仍有故障，603F=0x%04X", bus_->sdoRead(0x603Fu, 0)));
    }

    void servoEnable()
    {
        faultResetIfNeeded();
        bus_->commandState(0x0006u, 0x006Fu, 0x0021u, 1000, "Ready to switch on");
        bus_->commandState(0x0007u, 0x006Fu, 0x0023u, 1000, "Switched on");
        bus_->commandState(0x000Fu, 0x006Fu, 0x0027u, 1000, "Operation enabled");
    }

    void requireEnabled()
    {
        if ((bus_->statusWord() & 0x006Fu) != 0x0027u)
            throw std::runtime_error("电机未使能，先调 enable");
    }

    // ── 三模式点动 ────────────────────────────────────────────────────────────
    void movePP(double target_rad)
    {
        requireEnabled();
        const double vel = get_parameter("pp_profile_velocity").as_double();
        const double acc = get_parameter("pp_profile_accel").as_double();
        bus_->sdoWrite(0x6060u, 0, 1, 1);
        bus_->sdoWrite(0x6081u, 0, rb200().radToVelPP(vel), 4);
        bus_->sdoWrite(0x6083u, 0, rb200().radToVelPP(acc), 4);
        bus_->sdoWrite(0x6084u, 0, rb200().radToVelPP(acc), 4);
        const int32_t pp = rb200().radToPP(target_rad);
        bus_->sdoWrite(0x607Au, 0, static_cast<uint32_t>(pp), 4);
        bus_->sdoWrite(0x6040u, 0, 0x000Fu, 2);   // bit4 先置低（含清 halt）
        bus_->sdoWrite(0x6040u, 0, 0x003Fu, 2);   // bit4↑ 触发 + bit5 立即生效
        bus_->waitStatus(0x1000u, 0x1000u, 1000, "set-point 确认");
        bus_->sdoWrite(0x6040u, 0, 0x000Fu, 2);   // 清 bit4，等下一次触发
        logMsg(fmt("[ OK ] PP → %.4f rad（%d pp），轮廓速度 %.3f rad/s", target_rad, pp, vel));
    }

    void runPV(double rad_s)
    {
        requireEnabled();
        bus_->sdoWrite(0x60FFu, 0, 0, 4);          // 先清零再切模式，防跳变
        bus_->sdoWrite(0x6060u, 0, 3, 1);
        bus_->sdoWrite(0x6040u, 0, 0x000Fu, 2);    // 清 halt
        const int32_t pps = rb200().radToVelPPSigned(rad_s);
        bus_->sdoWrite(0x60FFu, 0, static_cast<uint32_t>(pps), 4);
        logMsg(fmt("[ OK ] PV → %.3f rad/s（%d pp/s），停止用 zero_target", rad_s, pps));
    }

    void runPT(double pct)
    {
        requireEnabled();
        bus_->sdoWrite(0x6071u, 0, 0, 2);
        bus_->sdoWrite(0x6060u, 0, 4, 1);
        bus_->sdoWrite(0x6040u, 0, 0x000Fu, 2);
        const auto permille = static_cast<int16_t>(pct * 10.0 + (pct >= 0 ? 0.5 : -0.5));
        bus_->sdoWrite(0x6071u, 0, static_cast<uint16_t>(permille), 2);
        logMsg(fmt("[ OK ] PT → %.1f %%（6071=%d）⚠️ 力矩模式「0」=自由下垂，不是停", pct, permille));
    }

    // ── 编码器置零（HM 方法 35，与 set_encoder_zero 同一流程）──────────────────
    const char * zeroCalibrate()
    {
        const int32_t before = bus_->sdoReadI32(0x6064u, 0);
        logMsg(fmt("置零前位置: %d pp（%.4f rad）", before, rb200().ppToRad(before)));
        servoEnable();
        bus_->sdoWrite(0x6060u, 0, 6, 1);     // HM
        bus_->sdoWrite(0x6098u, 0, 35, 1);    // 方法 35：当前位置即零点，无运动
        bus_->sdoWrite(0x607Cu, 0, 0, 4);
        bus_->sdoWrite(0x6099u, 1, 1, 4);
        bus_->sdoWrite(0x6099u, 2, 1, 4);
        bus_->sdoWrite(0x609Au, 0, 1, 4);
        try {
            bus_->commandState(0x001Fu, 0x1400u, 0x1400u, 3000, "回零完成");
        } catch (const std::exception & e) {
            const uint16_t sw = bus_->statusWord();
            const uint32_t fault = bus_->sdoRead(0x603Fu, 0);
            shutdown402();
            throw std::runtime_error(fmt("%s，603F=0x%04X%s", e.what(), fault,
                (sw & 0x2000u) ? "（bit13=1：回零错误，确认固件支持方法 35）" : ""));
        }
        bus_->sdoWrite(0x6040u, 0, 0x000Fu, 2);   // 清 bit4
        const int32_t after = bus_->sdoReadI32(0x6064u, 0);
        shutdown402();                             // 置零完成即失能
        if (after < -100 || after > 100)
            throw std::runtime_error(fmt("置零后位置 %d pp（理想 0，容差 ±100）", after));
        logMsg(fmt("[ OK ] 零点已写入编码器（断电保持），当前 %d pp", after));
        return "置零完成。请 NMT 复位或断电重启驱动器后验证位置 ≈ 0";
    }

    // ── 轮询反馈 ──────────────────────────────────────────────────────────────
    void poll()
    {
        try {
            bus_->heartbeat();
            const uint16_t sw   = bus_->statusWord();
            const int32_t  pos  = bus_->sdoReadI32(0x6064u, 0);
            const int32_t  vel  = bus_->sdoReadI32(0x606Cu, 0);
            const auto     tq   = static_cast<int16_t>(bus_->sdoRead(0x6077u, 0));
            const auto     mode = static_cast<int8_t>(bus_->sdoRead(0x6061u, 0));
            const uint16_t fault = (sw & 0x0008u)
                ? static_cast<uint16_t>(bus_->sdoRead(0x603Fu, 0)) : 0;
            int16_t cur = 0;   // 6078 实际电流（0.01A）;从站不支持时只降级、不掐轮询
            if (has_current_) {
                try {
                    cur = static_cast<int16_t>(bus_->sdoRead(0x6078u, 0));
                } catch (const std::exception & e) {
                    has_current_ = false;
                    logMsg(fmt("[WARN] 读电流 6078 失败（%s），电流显示停用", e.what()));
                }
            }

            sensor_msgs::msg::JointState js;
            js.header.stamp = now();
            js.name.push_back(joint_name_);
            js.position.push_back(rb200().ppToRad(pos));
            js.velocity.push_back(rb200().ppToRadS(vel));
            js.effort.push_back(tq / 10.0);
            state_pub_->publish(js);

            std_msgs::msg::UInt16MultiArray st;
            st.data = {sw, static_cast<uint16_t>(static_cast<uint8_t>(mode)), fault,
                       static_cast<uint16_t>(cur)};   // [3] 电流 0.01A，int16 按位打包
            status_pub_->publish(st);
        } catch (const std::exception & e) {
            poll_timer_->cancel();
            logMsg(fmt("[FAIL] 轮询中断：%s（检查供电/接线后重启本工具）", e.what()));
        }
    }

    // ── 基建 ──────────────────────────────────────────────────────────────────
    /** Trigger 服务模板：body 返回成功消息字符串，异常统一落 message + ~/log */
    void makeTrigger(const std::string & name, std::function<std::string()> body)
    {
        trigger_srvs_.push_back(create_service<Trigger>("~/" + name,
            [this, name, body = std::move(body)](Trigger::Request::ConstSharedPtr,
                                                 Trigger::Response::SharedPtr rs) {
                try {
                    rs->message = body();
                    rs->success = true;
                    logMsg("[ OK ] " + rs->message);
                } catch (const std::exception & e) {
                    rs->message = e.what();
                    rs->success = false;
                    logMsg(fmt("[FAIL] %s：%s", name.c_str(), e.what()));
                }
            }));
    }

    bool guard(const char * op, const std::function<void()> & body)
    {
        try {
            body();
            return true;
        } catch (const std::exception & e) {
            logMsg(fmt("[FAIL] %s：%s", op, e.what()));
            return false;
        }
    }

    void logMsg(const std::string & s)
    {
        RCLCPP_INFO(get_logger(), "%s", s.c_str());
        std_msgs::msg::String m;
        m.data = s;
        log_pub_->publish(m);
    }

    template <typename... Args>
    static std::string fmt(const char * f, Args... args)
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf), f, args...);
        return buf;
    }

    std::unique_ptr<SdoClient> bus_;
    std::string joint_name_;
    bool has_current_ = true;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    std::vector<rclcpp::Service<Trigger>::SharedPtr> trigger_srvs_;
    rclcpp::Service<COTargetDouble>::SharedPtr move_pp_srv_, run_pv_srv_, run_pt_srv_;
    rclcpp::Service<COReadID>::SharedPtr sdo_read_srv_;
    rclcpp::Service<COWriteID>::SharedPtr sdo_write_srv_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        auto node = std::make_shared<MotorDebugBackend>();
        rclcpp::spin(node);          // Ctrl-C / launch 关停 → spin 返回
        node->safeShutdown();        // 显式失能（析构还会兜一次，幂等）
    } catch (const std::exception & e) {
        std::fprintf(stderr, "motor_debug_backend 启动失败：%s\n"
                     "检查：CAN 口是否拉起 / 节点号 / 供电 / ros2_control 栈是否已停。\n",
                     e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
