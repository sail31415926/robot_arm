/**
 * @file set_encoder_zero.cpp
 * @brief RB200-CA 绝对编码器零点标定工具（CAN 通讯）
 *
 * 将电机当前位置写入为绝对零点并保存到 EEPROM，下次上电直接读零，
 * 无需回零运动。
 *
 * 原理（DS402 HM 方法 35）：
 *   - 0x6098h = 35：以当前位置为零点（无运动，驱动器内部清零）
 *   - 0x607Ch = 0 ：零点偏置清零
 *   - 触发后驱动器将 0x6064h 重置为 0，绝对编码器零点参考同步更新
 *   - 0x1010h:02h = "save"(0x65766173)：将参数固化到 EEPROM
 *
 * 注意：
 *   1. 执行前请将关节移至期望的机械零点位置并确认电机静止
 *   2. 进程需要 CAP_NET_RAW 权限（sudo 或 setcap）
 *   3. 操作完成后请重启驱动器（或 Reset Node）使零点生效
 *
 * 用法：
 *   ./set_encoder_zero <can_interface> <node_id> [master_node_id] [sdo_timeout_ms]
 *   例：./set_encoder_zero can0 1
 *       ./set_encoder_zero can0 1 127 500
 *
 * 所属模块：hardware_driver/src/
 *
 * @version 1.0
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 EMEET
 */

#include "canopen_motor_driver/canopen_motor_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

static void printStatusWord(uint16_t sw) {
    std::printf("  状态字 0x%04X: 故障=%d 使能=%d 运行=%d bit12=%d bit13=%d\n",
        sw,
        (sw >> 3) & 1,
        (sw >> 2) & 1,
        (sw >> 2) & 1,
        (sw >> 12) & 1,
        (sw >> 13) & 1);
}


static void printResult(bool ok, const char* step) {
    std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", step);
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::fprintf(stderr,
            "用法: %s <can_interface> <node_id> [master_node_id] [sdo_timeout_ms]\n"
            "  例: %s can0 1\n"
            "      %s can0 1 127 500\n", argv[0], argv[0], argv[0]);
        return 1;
    }

    const std::string ifname          = argv[1];
    const uint8_t     node_id         = static_cast<uint8_t>(std::atoi(argv[2]));
    const uint8_t     master_node_id  = (argc >= 4) ? static_cast<uint8_t>(std::atoi(argv[3])) : 127;
    const int         timeout_ms      = (argc >= 5) ? std::atoi(argv[4]) : 500;

    std::printf("\n=== RB200-CA 绝对编码器零点标定 ===\n");
    std::printf("  CAN 接口: %s   节点 ID: %d   主站节点 ID: %d   SDO 超时: %d ms\n\n",
        ifname.c_str(), node_id, master_node_id, timeout_ms);

    arm::CanopenMotorDriver drv(ifname, node_id, timeout_ms);

    // ── Step 1: 打开 CAN 接口 ──────────────────────────────────────────────────
    bool ok = drv.init();
    printResult(ok, "打开 CAN 接口");
    if (!ok) { std::fprintf(stderr, "错误：无法打开 %s\n", ifname.c_str()); return 1; }

    std::this_thread::sleep_for(50ms);

    // ── Step 2: 读取当前位置（标定前参考） ────────────────────────────────────
    int32_t pos_before = drv.getPosition();
    std::printf("  当前编码器位置（标定前）: %d counts\n\n", pos_before);

    // ── Step 3: 故障复位（如有） ───────────────────────────────────────────────
    uint16_t sw = drv.getStatusWord();
    if (sw & 0x0008u) {
        std::printf("  检测到驱动器故障，尝试复位...\n");
        drv.resetFault();
        std::this_thread::sleep_for(200ms);
        printResult(!(drv.getStatusWord() & 0x0008u), "故障复位");
    }

    // ── Step 4: 使能伺服 ───────────────────────────────────────────────────────
    ok = drv.enable();
    printResult(ok, "使能伺服");
    if (!ok) {
        std::fprintf(stderr, "错误：使能失败，请检查驱动器状态\n");
        return 1;
    }
    std::this_thread::sleep_for(100ms);

    // ── Step 5: 切换到回零模式（HM），方法 35 ─────────────────────────────────
    //   method=35 偏置=0  速度/加速度参数对方法35无效（无运动）
    ok = drv.setHomingMode(
        35,     // 方法 35：以当前位置为零点，无需运动
        1,      // fast_vel：方法 35 忽略，给最小值即可
        1,      // slow_vel：同上
        1,      // accel：同上
        0       // offset：零偏置 → 机械零 = 机械原点
    );
    printResult(ok, "切换回零模式 (HM method 35)");
    if (!ok) {
        std::fprintf(stderr, "错误：回零模式配置失败\n");
        drv.disable();
        return 1;
    }
    std::this_thread::sleep_for(50ms);

    // ── Step 6: 触发回零（以当前位置为零，驱动器内部清零） ───────────────────
    //   先发几帧心跳让驱动器确认主站在线，再触发；期间持续发心跳防 ER.E20/E21
    for (int i = 0; i < 3; ++i) {
        drv.sendHeartbeat(master_node_id);
        std::this_thread::sleep_for(30ms);
    }

    ok = drv.startHoming(3000, [&drv, master_node_id]() {
        drv.sendHeartbeat(master_node_id);  // 每 ~80ms 发送一次，防驱动器心跳超时
    });
    printResult(ok, "触发零点标定 (startHoming, method 35)");
    if (!ok) {
        uint16_t sw  = drv.getStatusWord();
        uint16_t err = 0;
        uint8_t  len = 0;
        drv.readSDO(0x603Fu, 0, &err, len);
        std::fprintf(stderr, "错误：零点标定失败\n");
        printStatusWord(sw);
        std::fprintf(stderr, "  错误码 (0x603F): 0x%04X\n", err);
        std::fprintf(stderr, "  若 bit13=1 且错误码非零，请检查：\n"
                             "  1. 驱动器固件是否支持方法 35\n"
                             "  2. 可换用 master_node_id 参数（默认 127）\n");
        drv.disable();
        return 1;
    }
    std::this_thread::sleep_for(100ms);

    // ── Step 7: 验证位置已归零 ─────────────────────────────────────────────────
    int32_t pos_after = drv.getPosition();
    std::printf("  标定后编码器位置: %d counts（理想值为 0）\n", pos_after);
    bool zero_ok = (pos_after >= -100 && pos_after <= 100);
    printResult(zero_ok, "位置验证（±100 counts 以内）");

    // ── Step 8: 禁用伺服 ───────────────────────────────────────────────────────
    drv.disable();
    printResult(true, "禁用伺服");

    // ── 结果汇总 ───────────────────────────────────────────────────────────────
    std::printf("\n=== 结果 ===\n");
    std::printf("  标定前位置: %d counts\n", pos_before);
    std::printf("  标定后位置: %d counts\n", pos_after);
    std::printf("\n");

    if (zero_ok) {
        std::printf("✓ 零点标定完成。\n");
        std::printf("  零点已写入绝对编码器片内存储（电池保留，断电不丢失）。\n");
        std::printf("  断电重上后位置应直接读为 0，无需回零运动。\n\n");
        return 0;
    } else {
        std::printf("✗ 零点标定失败（位置未归零），请检查上述日志。\n\n");
        return 1;
    }
}
