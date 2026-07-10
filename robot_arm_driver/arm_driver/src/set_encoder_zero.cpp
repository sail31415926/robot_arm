/**
 * @file set_encoder_zero.cpp
 * @brief RB200-CA 绝对编码器零点快速标定工具（HM 方法 35，独立 SocketCAN）
 *
 * 把电机「当前位置」设为绝对零点，写入编码器片内存储（电池保持，断电不丢失），
 * 下次上电直接读零、无需回零运动。移植自旧栈同名工具（实测可用），标零机制为
 * DS402 标准回零方法 35：
 *   6060h=6 (HM) → 6098h=35 (以当前位置为零点，无运动) → 607Ch=0 (零偏置)
 *   → 控制字 bit4 上升沿触发 → 状态字 bit12(回零完成)+bit10(到达) → 6064h 归零
 *
 * 独立工具（自带极简 SDO 客户端 + 主站心跳，不依赖 ros2_canopen 运行时）。
 * ⚠️ 运行前必须先停掉 ros2_control 栈（test_arm / real.launch），
 *    否则主站 RPDO 的控制字会和本工具打架。
 *
 * 用法（关节摆到机械零位、静止后执行）：
 *   ros2 run robot_arm_driver set_encoder_zero <can接口> <node_id...> [master_id] [sdo超时ms]
 *   例：ros2 run robot_arm_driver set_encoder_zero can0 1          # 只标 J1
 *       ros2 run robot_arm_driver set_encoder_zero can0 1 2 3     # 三关节依次标零
 *       ros2 run robot_arm_driver set_encoder_zero can0 1 2 3 127 500
 *
 * 标完后重启驱动器（断电重上或 NMT Reset），再起栈确认 /joint_states ≈ 0。
 *
 * @version 2.0（新栈移植版；旧版基于 canopen_motor_driver，已随自研栈下线）
 * @date 2026-07-08
 * @copyright Copyright (c) 2026 EMEET
 */

#include "sdo_client.hpp"

#include <cstdlib>
#include <utility>
#include <vector>

using arm_tools::SdoClient;
using arm_tools::rb200;
using namespace std::chrono_literals;

// ─── 标零流程（单节点）────────────────────────────────────────────────────────
static void stepOk(const char * msg) { std::printf("  [ OK ] %s\n", msg); }

static bool setZero(SdoClient & bus)
{
    int32_t pos_before = bus.sdoReadI32(0x6064u, 0);
    std::printf("  标定前位置: %d counts (%+.4f rad)\n",
                pos_before, rb200().ppToRad(pos_before));

    // 故障复位（如有）：控制字 bit7 上升沿
    if (bus.statusWord() & 0x0008u) {
        std::printf("  检测到故障，先复位...\n");
        bus.sdoWrite(0x6040u, 0, 0x0080u, 2);
        std::this_thread::sleep_for(200ms);
        bus.sdoWrite(0x6040u, 0, 0x0000u, 2);
        std::this_thread::sleep_for(200ms);
        if (bus.statusWord() & 0x0008u) {
            std::printf("  [FAIL] 故障复位失败，错误码 0x603F=0x%04X\n", bus.sdoRead(0x603Fu, 0));
            return false;
        }
        stepOk("故障复位");
    }

    // 402 使能序列：Shutdown → Switch On → Enable Operation
    bus.commandState(0x0006u, 0x006Fu, 0x0021u, 1000, "Ready to switch on");
    bus.commandState(0x0007u, 0x006Fu, 0x0023u, 1000, "Switched on");
    bus.commandState(0x000Fu, 0x006Fu, 0x0027u, 1000, "Operation enabled");
    stepOk("使能伺服");

    // HM 方法 35 配置：无运动，速度/加速度给最小值即可
    bus.sdoWrite(0x6060u, 0, 6, 1);    // 模式 = HM
    bus.sdoWrite(0x6098u, 0, 35, 1);   // 方法 35：以当前位置为零点
    bus.sdoWrite(0x607Cu, 0, 0, 4);    // 零点偏置 = 0（机械零 = 机械原点）
    bus.sdoWrite(0x6099u, 1, 1, 4);
    bus.sdoWrite(0x6099u, 2, 1, 4);
    bus.sdoWrite(0x609Au, 0, 1, 4);
    stepOk("配置回零模式 (HM method 35)");

    // 先发几帧心跳让驱动器确认主站在线，再触发（控制字 bit4 上升沿）
    for (int i = 0; i < 3; ++i) {
        bus.heartbeat();
        std::this_thread::sleep_for(30ms);
    }
    try {
        // 控制字 bit4=1 触发；重复写保持电平（等 bit12 回零完成 + bit10 到达）
        bus.commandState(0x001Fu, 0x1400u, 0x1400u, 3000, "回零完成");
    } catch (const std::exception & e) {
        uint16_t sw  = bus.statusWord();
        std::printf("  [FAIL] %s\n", e.what());
        std::printf("         错误码 0x603F=0x%04X\n", bus.sdoRead(0x603Fu, 0));
        if (sw & 0x2000u)
            std::printf("         bit13=1：回零错误——确认驱动器固件支持方法 35\n");
        bus.sdoWrite(0x6040u, 0, 0x0006u, 2);
        return false;
    }
    bus.sdoWrite(0x6040u, 0, 0x000Fu, 2);  // 清 bit4
    stepOk("触发零点标定");

    int32_t pos_after = bus.sdoReadI32(0x6064u, 0);
    bool ok = (pos_after >= -100 && pos_after <= 100);
    std::printf("  标定后位置: %d counts（理想 0，容差 ±100）\n", pos_after);

    bus.sdoWrite(0x6040u, 0, 0x0006u, 2);  // 失能（Shutdown）
    stepOk("禁用伺服");
    return ok;
}

int main(int argc, char * argv[])
{
    if (argc < 3) {
        std::fprintf(stderr,
            "用法: %s <can接口> <node_id...> [master_id=127] [sdo超时ms=500]\n"
            "  例: %s can0 1\n"
            "      %s can0 1 2 3\n"
            "      %s can0 1 2 3 127 500\n"
            "注意: 运行前先停掉 ros2_control 栈，并把关节摆到机械零位。\n",
            argv[0], argv[0], argv[0], argv[0]);
        return 1;
    }

    const std::string ifname = argv[1];
    std::vector<uint8_t> nodes;
    int master_id = 127, timeout_ms = 500;

    // 位置参数：连续的 node_id，之后可选 master_id、超时（node_id 均 <64 可与 127 区分）
    std::vector<int> tail;
    for (int i = 2; i < argc; ++i) tail.push_back(std::atoi(argv[i]));
    size_t n_nodes = tail.size();
    if (n_nodes >= 2 && tail[n_nodes - 2] >= 64) { timeout_ms = tail.back(); master_id = tail[n_nodes - 2]; n_nodes -= 2; }
    else if (n_nodes >= 1 && tail[n_nodes - 1] >= 64) { master_id = tail.back(); n_nodes -= 1; }
    for (size_t i = 0; i < n_nodes; ++i) nodes.push_back(static_cast<uint8_t>(tail[i]));
    if (nodes.empty()) { std::fprintf(stderr, "错误：未给出有效 node_id（1~63）\n"); return 1; }

    std::printf("\n=== RB200-CA 绝对编码器零点标定（HM 方法 35）===\n");
    std::printf("  接口: %s   主站 ID: %d   SDO 超时: %dms\n", ifname.c_str(), master_id, timeout_ms);
    std::printf("⚠️  确认：① ros2_control 栈已停止  ② 关节已摆到机械零位且静止\n\n");

    bool all_ok = true;
    std::vector<std::pair<int, bool>> results;
    for (uint8_t node : nodes) {
        std::printf("── 节点 %d ──────────────────────────────\n", node);
        bool ok = false;
        try {
            SdoClient bus(ifname, node, static_cast<uint8_t>(master_id), timeout_ms);
            ok = setZero(bus);
        } catch (const std::exception & e) {
            std::printf("  [FAIL] %s\n", e.what());
        }
        results.emplace_back(node, ok);
        all_ok &= ok;
        std::printf("\n");
    }

    std::printf("=== 结果 ===\n");
    for (auto & [node, ok] : results)
        std::printf("  节点 %d: %s\n", node, ok ? "✓ 零点已写入（断电保持）" : "✗ 失败");
    if (all_ok)
        std::printf("\n全部完成。重启驱动器后起栈，确认 /joint_states ≈ 0。\n\n");
    else
        std::printf("\n存在失败项，见上方日志。\n\n");
    return all_ok ? 0 : 1;
}
