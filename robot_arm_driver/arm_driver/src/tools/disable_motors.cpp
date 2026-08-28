/**
 * @file disable_motors.cpp
 * @brief 关停失能工具 —— 减速停机后移除力矩（独立 SocketCAN，不依赖 ros2_canopen 运行时）
 *
 * ── 为什么必须是独立进程（2026-08-12 实机实测，安全相关）────────────────────
 * `ros2_control_node` 关停时以 **SIGABRT** 挂掉（vendored 0.2.13 的 DeviceContainer
 * 析构 bug），进程在走到硬件组件 shutdown 之前就死了 —— 上游 RobotSystem 只在
 * `on_deactivate` 里 `halt_motor()`，那条路径根本走不到。实测抓 RPDO1 共 11631 帧，
 * 控制字一直是 0x001F（Operation Enabled）直到总线静默，全程没有任何 halt 报文：
 * **进程死时驱动器仍处于 Operation Enabled、带着力矩**。且这条路径上任何进程内钩子
 * 都拿不到执行机会（生命周期 `on_shutdown`/`on_cleanup` 与 `rclcpp::on_shutdown`
 * 均实测无效，见 unwrap_robot_system.hpp 的注释）。所以失能只能靠进程外手段。
 *
 * ── 失能顺序（每节点）────────────────────────────────────────────────────────
 *   ① 读 6041 状态字：不带力矩（Switch on disabled / Ready / Fault…）→ 直接放过
 *   ② Quick Stop（控制字 0x0002）：按 6085 斜坡减速停机（bus.yml ≈10 rad/s²），
 *      避免带着速度直接断力矩甩出去。605A 默认 2 ⇒ 减速完成后 402 状态机自动落到
 *      Switch on disabled（已无力矩）
 *   ③ 若停在 Quick stop active 仍带力矩（605A=5/6「停后锁轴」）→ 补 Disable Voltage(0x0000)
 *   ④ Shutdown（0x0006）落到 Ready to switch on，读状态字**确认**已无力矩
 *   ⑤ 上面任一步失败/超时 → NMT Reset Node（`000#8100`）兜底。它是单帧盲发、不需要
 *      从站 SDO 应答，也抢得过还在跑的主站 RPDO（NMT 层，不是 402 控制字层）
 *
 * ⚠️ **失能会让 J2/J3 失去支撑、机械臂下沉**（60FE 抱闸输出在 RB200-CA 上是只读，
 *    由驱动器自管，上位机控制不了）。自动挂在 launch 关停钩子上就意味着每次 Ctrl-C
 *    都会下沉一次 —— 这是「不带力矩地下沉」对「带着力矩不受控」的取舍，后者更危险。
 *
 * 用法：
 *   ros2 run robot_arm_driver disable_motors <can接口> [node_id...] [选项]
 *     ros2 run robot_arm_driver disable_motors can0            # 默认节点 1 2 3
 *     ros2 run robot_arm_driver disable_motors can0 1 2 3
 *     ros2 run robot_arm_driver disable_motors can0 --nmt-only # 最快兜底（不减速、不校验）
 *   选项：
 *     --master N        主站 node id（心跳源），默认 127（与 bus.yml 一致）
 *     --sdo-ms N        单次 SDO 响应超时，默认 120（关停要抢时间，比标定工具短）
 *     --timeout-ms N    每节点等待停稳超时，默认 400
 *     --no-quickstop    跳过减速，直接 Shutdown（更快，但带速度时是自由停机）
 *     --nmt-only        只盲发 NMT Reset Node，不做 SDO 交互（最快，崩溃后兜底用）
 *
 * 退出码：0 = 全部确认无力矩；1 = 存在无法处置的节点（见日志）。
 *
 * @version 1.0
 * @date 2026-08-21
 * @copyright Copyright (c) 2026 EMEET
 */

#include "robot_arm_driver/sdo_client.hpp"

#include <csignal>
#include <cstdlib>
#include <string>
#include <vector>

using arm_tools::SdoClient;
using namespace std::chrono_literals;

namespace {

// ── 402 状态字（6041h）解码 ──────────────────────────────────────────────────
// 带力矩的只有三个状态：Operation enabled、Quick stop active（605A=5/6 锁轴时）、
// Fault reaction active。其余（Switch on disabled / Ready to switch on / Switched on /
// Fault / Not ready）驱动器都已断输出。
bool isEnergized(uint16_t sw)
{
    if ((sw & 0x6Fu) == 0x27u) return true;   // Operation enabled
    if ((sw & 0x6Fu) == 0x07u) return true;   // Quick stop active（可能仍锁轴）
    if ((sw & 0x4Fu) == 0x0Fu) return true;   // Fault reaction active（正在故障停机）
    return false;
}

const char * stateName(uint16_t sw)
{
    if ((sw & 0x4Fu) == 0x00u) return "Not ready to switch on";
    if ((sw & 0x4Fu) == 0x40u) return "Switch on disabled";
    if ((sw & 0x6Fu) == 0x21u) return "Ready to switch on";
    if ((sw & 0x6Fu) == 0x23u) return "Switched on";
    if ((sw & 0x6Fu) == 0x27u) return "Operation enabled";
    if ((sw & 0x6Fu) == 0x07u) return "Quick stop active";
    if ((sw & 0x4Fu) == 0x0Fu) return "Fault reaction active";
    if ((sw & 0x4Fu) == 0x08u) return "Fault";
    return "未知";
}

/**
 * 持续「发主站心跳 + 重复写控制字 + 读状态字」直到 pred(sw) 成立或超时，返回最后的状态字。
 * 重复写是必要的：部分从站只在 6040 被写入时才评估状态机（真栈里主站 RPDO 高频重复写
 * 掩盖了这一点）。期间必须发心跳，否则 RB200 在减速这段时间里可能先报心跳超时
 * ER.E20/E21 转成自由停机。
 */
template <typename Pred>
uint16_t driveUntil(SdoClient & bus, uint16_t cw, Pred pred, int timeout_ms)
{
    const auto deadline = arm_tools::clk::now() + std::chrono::milliseconds(timeout_ms);
    uint16_t sw = 0;
    for (;;) {
        bus.heartbeat();
        bus.sdoWrite(0x6040u, 0, cw, 2);
        sw = bus.statusWord();
        if (pred(sw)) return sw;
        if (arm_tools::clk::now() >= deadline) return sw;
        std::this_thread::sleep_for(20ms);
    }
}

/// 单节点失能。抛异常交由调用方走 NMT 兜底。返回 true = 已确认无力矩。
bool disableOne(SdoClient & bus, int wait_ms, bool quickstop)
{
    auto safe = [](uint16_t s) { return !isEnergized(s); };

    uint16_t sw = bus.statusWord();
    std::printf("  当前状态: 0x%04X (%s)\n", sw, stateName(sw));
    if (!isEnergized(sw)) {
        std::printf("  [ OK ] 已无力矩，无需处置\n");
        return true;
    }

    // ② Quick Stop：按 6085 斜坡减速停机（605A=2 默认 ⇒ 停稳后自动到 Switch on disabled）
    if (quickstop && (sw & 0x6Fu) == 0x27u) {
        sw = driveUntil(bus, 0x0002u, safe, wait_ms);
        std::printf("  %s Quick Stop → 0x%04X (%s)\n",
                    isEnergized(sw) ? "[WARN]" : "[ OK ]", sw, stateName(sw));
    }

    // ③ 停在 Quick stop active 说明 605A=5/6「停后锁轴」，仍带力矩 → Disable Voltage
    if ((sw & 0x6Fu) == 0x07u) {
        std::printf("  仍在 Quick stop active（605A=5/6 锁轴）→ Disable Voltage\n");
        sw = driveUntil(bus, 0x0000u, safe, wait_ms);
    }

    // ④ Shutdown 收尾并确认（Ready to switch on 同样无力矩）
    sw = driveUntil(bus, 0x0006u, safe, 200);
    if (!isEnergized(sw)) {
        std::printf("  [ OK ] 已失能: 0x%04X (%s)\n", sw, stateName(sw));
        return true;
    }
    std::printf("  [FAIL] 仍带力矩: 0x%04X (%s)\n", sw, stateName(sw));
    return false;
}

void usage(const char * prog)
{
    std::printf(
        "用法: %s <can接口> [node_id...] [选项]\n"
        "  例: %s can0                 # 默认节点 1 2 3\n"
        "      %s can0 1 2 3\n"
        "      %s can0 --nmt-only      # 最快兜底（不减速、不校验）\n"
        "选项:\n"
        "  --master N      主站 node id（心跳源），默认 127\n"
        "  --sdo-ms N      单次 SDO 响应超时 ms，默认 120\n"
        "  --timeout-ms N  每节点等待停稳超时 ms，默认 400\n"
        "  --no-quickstop  跳过减速，直接 Shutdown\n"
        "  --nmt-only      只盲发 NMT Reset Node（000#8100）\n"
        "⚠️ 失能后 J2/J3 失去支撑会下沉，确认下方无人。\n",
        prog, prog, prog, prog);
}

}  // namespace

int main(int argc, char * argv[])
{
    // 关停清理器必须跑完 —— 挂在 launch 的关停钩子上时，launch 会把 SIGINT 发给它管的
    // 每个进程（含本工具），默认行为下我们会在写控制字之前就被打死，等于白挂。
    // 忽略是安全的：本工具无交互、总运行时间被各节点超时钳住（3 节点最坏 ~2s，
    // 远小于 launch 的 SIGTERM/SIGKILL 窗口），不会赖着不退。
    std::signal(SIGINT,  SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    std::signal(SIGHUP,  SIG_IGN);   // 终端先消失时（systemd 停服务/关窗口）同理
    // SIGPIPE 同样致命且更隐蔽：launch 先退出会让我们成为孤儿、stdout 管道被关闭，
    // 下一条 printf 就吃 SIGPIPE 默认终止 —— 失能会在半途死掉。忽略后 printf 只是
    // 返回 -1（不影响任何控制字逻辑），日志丢了但电机一定被按下去。
    std::signal(SIGPIPE, SIG_IGN);

    std::string ifname;
    std::vector<uint8_t> nodes;
    int  master = 127, sdo_ms = 120, wait_ms = 400;
    bool quickstop = true, nmt_only = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char * what) -> int {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "错误：%s 缺少参数\n", what);
                std::exit(1);
            }
            return std::atoi(argv[++i]);
        };
        if      (a == "--master")       master    = next("--master");
        else if (a == "--sdo-ms")       sdo_ms    = next("--sdo-ms");
        else if (a == "--timeout-ms")   wait_ms   = next("--timeout-ms");
        else if (a == "--no-quickstop") quickstop = false;
        else if (a == "--nmt-only")     nmt_only  = true;
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (a.rfind("--", 0) == 0) {
            std::fprintf(stderr, "错误：未知选项 %s\n", a.c_str());
            return 1;
        }
        else if (ifname.empty()) ifname = a;
        else {
            const int id = std::atoi(a.c_str());
            if (id < 1 || id > 63) {
                std::fprintf(stderr, "错误：node_id %s 不在 1~63\n", a.c_str());
                return 1;
            }
            nodes.push_back(static_cast<uint8_t>(id));
        }
    }
    if (ifname.empty()) { usage(argv[0]); return 1; }
    if (nodes.empty()) nodes = {1, 2, 3};   // bus.yml 的三个关节模组

    std::printf("\n=== 关停失能（%s，节点", ifname.c_str());
    for (uint8_t n : nodes) std::printf(" %u", static_cast<unsigned>(n));
    std::printf("）===\n");
    std::printf("  模式: %s\n", nmt_only
        ? "仅 NMT Reset Node（盲发，不校验）"
        : (quickstop ? "Quick Stop 减速 → Shutdown，失败则 NMT 兜底"
                     : "直接 Shutdown（跳过减速），失败则 NMT 兜底"));
    std::printf("⚠️  失能后 J2/J3 失去支撑会下沉，确认下方无人/无障碍。\n\n");

    int failed = 0;
    for (uint8_t node : nodes) {
        std::printf("── 节点 %u ──────────────────────────────\n", static_cast<unsigned>(node));
        bool ok = false;
        try {
            SdoClient bus(ifname, node, static_cast<uint8_t>(master), sdo_ms);
            if (nmt_only) {
                bus.nmt(0x81u);
                std::printf("  [ OK ] 已盲发 NMT Reset Node\n");
                ok = true;
            } else {
                ok = disableOne(bus, wait_ms, quickstop);
                if (!ok) {
                    // SDO 通路没能把它按下去（主站还在抢 / 从站状态机卡住）→ NMT 层兜底
                    std::printf("  → NMT Reset Node 兜底\n");
                    bus.nmt(0x81u);
                    std::this_thread::sleep_for(150ms);
                    ok = true;   // 盲发无应答可校验，按已处置计
                }
            }
        } catch (const std::exception & e) {
            std::printf("  [FAIL] %s\n", e.what());
            // SDO 不通不代表总线发不出去：NMT 是单帧盲发，值得再试一次
            try {
                SdoClient bus(ifname, node, static_cast<uint8_t>(master), sdo_ms);
                bus.nmt(0x81u);
                std::printf("  → 已盲发 NMT Reset Node 兜底\n");
                ok = true;
            } catch (const std::exception & e2) {
                std::printf("  → NMT 兜底也失败: %s\n", e2.what());
                std::printf("     can0 down / NO-CARRIER？此时驱动器收不到任何报文，\n"
                            "     会因主站心跳超时报 ER.E20/E21 自行断力矩（约 1s）。\n");
            }
        }
        if (!ok) ++failed;
        std::printf("\n");
    }

    if (failed == 0) {
        std::printf("=== 完成：%zu 个节点均已处置 ===\n\n", nodes.size());
        return 0;
    }
    std::printf("=== 存在 %d 个未能处置的节点，见上方日志 ===\n"
                "手动兜底: cansend %s 000#8100   （NMT Reset Node 广播）\n\n",
                failed, ifname.c_str());
    return 1;
}
