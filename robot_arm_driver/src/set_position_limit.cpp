/**
 * @file set_position_limit.cpp
 * @brief RB200-CA 软件位置限位标定工具（CAN 通讯）
 *
 * 将关节当前位置写入为正向 / 负向软件限位（DS402 0x607Dh）。
 * 关节运动到限位时驱动器会停止并报警，是机械臂安全配置的关键一步。
 *
 * 对象字典：
 *   0x607D:01h  最小绝对位置限制（负向最大值）INT32  指令单位
 *   0x607D:02h  最大绝对位置限制（正向最大值）INT32  指令单位
 *   两个都是默认值（-2^31 / 2^31-1）时软件限位不生效
 *
 * 推荐操作流程：
 *   1. 先用 set_encoder_zero 标定零点并断电重上
 *   2. 手动将关节分别转到负向 / 正向最远位置，记录 counts 值（用 show 查看）
 *   3. 一次写入两端：set_position_limit can0 1 set <neg_counts> <pos_counts>
 *   4. 用 VCSDSoft_L「参数管理 → 保存到 EEPROM」固化（驱动器不支持 SDO 标准保存）
 *
 * 用法：
 *   ./set_position_limit <can_interface> <node_id> <set|pos|neg|show|clear> [neg_counts pos_counts]
 *   例：./set_position_limit can0 1 set -66780 247736  # 一次写入两端限位
 *       ./set_position_limit can0 1 pos      # 当前位置为正限位（单侧）
 *       ./set_position_limit can0 1 neg      # 当前位置为负限位（单侧）
 *       ./set_position_limit can0 1 show     # 显示当前限位
 *       ./set_position_limit can0 1 clear    # 重置到默认（关闭限位）
 *
 * 所属模块：hardware_driver/src/
 *
 * @version 1.1
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 EMEET
 */

#include "canopen_motor_driver/canopen_motor_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

static constexpr int32_t INT32_MIN_V = -2147483647 - 1;  // -2^31
static constexpr int32_t INT32_MAX_V =  2147483647;      //  2^31-1
static constexpr int64_t CPR_DEFAULT = 524288;            // RB200-CA JM 编码器一圈

static bool readLimit(arm::CanopenMotorDriver& drv, uint8_t sub, int32_t& out) {
    uint8_t buf[4]{}; uint8_t len = 0;
    if (!drv.readSDO(0x607Du, sub, buf, len)) return false;
    out = int32_t(uint32_t(buf[0])
        | uint32_t(buf[1]) << 8
        | uint32_t(buf[2]) << 16
        | uint32_t(buf[3]) << 24);
    return true;
}

static bool writeLimit(arm::CanopenMotorDriver& drv, uint8_t sub, int32_t v) {
    return drv.writeSDO(0x607Du, sub, &v, 4);
}

// CiA301 0x1010:01 — Save all objects to EEPROM
static bool saveToEEPROM(arm::CanopenMotorDriver& drv) {
    const uint32_t save_sig = 0x65766173u;  // ASCII "save" little-endian
    return drv.writeSDO(0x1010u, 0x01u, &save_sig, 4);
}

static void printLimit(const char* name, int32_t v, int32_t default_v) {
    double rad = double(v) / CPR_DEFAULT * 2.0 * M_PI;
    bool is_default = (v == default_v);
    std::printf("  %s: %d counts (%.4f rad)%s\n",
        name, v, rad, is_default ? "   [默认值，限位未启用]" : "");
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::fprintf(stderr,
            "用法: %s <can_interface> <node_id> <set|pos|neg|show|clear> [neg_counts pos_counts]\n"
            "  set <neg> <pos>  一次写入负向和正向限位（推荐，避免固件单侧写入重置问题）\n"
            "  pos              把当前位置写入正向限位 (0x607D:02)\n"
            "  neg              把当前位置写入负向限位 (0x607D:01)\n"
            "  show             显示当前限位值\n"
            "  clear            恢复默认值（关闭限位监控）\n"
            "  例: %s can0 1 set -66780 247736\n", argv[0], argv[0]);
        return 1;
    }

    const std::string ifname  = argv[1];
    const uint8_t     node_id = static_cast<uint8_t>(std::atoi(argv[2]));
    const std::string action  = argv[3];

    std::printf("\n=== RB200-CA 软件位置限位标定 ===\n");
    std::printf("  CAN: %s   节点 ID: %d   操作: %s\n\n",
        ifname.c_str(), node_id, action.c_str());

    arm::CanopenMotorDriver drv(ifname, node_id, 500);
    if (!drv.init()) {
        std::fprintf(stderr, "错误: 无法打开 CAN 接口 %s\n", ifname.c_str());
        return 1;
    }
    std::this_thread::sleep_for(50ms);

    // 读取当前限位与位置
    int32_t min_lim = 0, max_lim = 0;
    if (!readLimit(drv, 0x01, min_lim) || !readLimit(drv, 0x02, max_lim)) {
        std::fprintf(stderr, "错误: 读取 0x607Dh 失败\n");
        return 1;
    }
    int32_t cur_pos = drv.getPosition();

    std::printf("当前状态：\n");
    std::printf("  实际位置: %d counts (%.4f rad)\n",
        cur_pos, double(cur_pos) / CPR_DEFAULT * 2.0 * M_PI);
    printLimit("负向限位 (607D:01)", min_lim, INT32_MIN_V);
    printLimit("正向限位 (607D:02)", max_lim, INT32_MAX_V);
    std::printf("\n");

    // 若已存在的限位本身就不合法（min >= max 且都非默认），跳过交叉安全检查，
    // 给用户机会修复
    bool limits_already_invalid =
        (min_lim != INT32_MIN_V && max_lim != INT32_MAX_V && min_lim >= max_lim);
    if (limits_already_invalid) {
        std::printf("⚠ 检测到当前限位已不合法（min >= max），跳过安全检查以便修复。\n");
        std::printf("  建议：先 `clear` 重置，再分别在两端重新设置。\n\n");
    }

    bool ok = true;
    if (action == "set") {
        if (argc < 6) {
            std::fprintf(stderr, "错误: set 操作需要两个参数: <neg_counts> <pos_counts>\n"
                "  例: %s can0 1 set -66780 247736\n", argv[0]);
            return 1;
        }
        int32_t new_neg = static_cast<int32_t>(std::atol(argv[4]));
        int32_t new_pos = static_cast<int32_t>(std::atol(argv[5]));
        if (new_neg >= new_pos) {
            std::fprintf(stderr, "错误: 负向限位 (%d) 必须小于正向限位 (%d)\n",
                new_neg, new_pos);
            return 1;
        }
        bool ok1 = writeLimit(drv, 0x01, new_neg);
        std::printf("写入负向限位: %d counts (%.4f rad) → %s\n",
            new_neg, double(new_neg) / CPR_DEFAULT * 2.0 * M_PI, ok1 ? "成功" : "失败");
        bool ok2 = writeLimit(drv, 0x02, new_pos);
        std::printf("写入正向限位: %d counts (%.4f rad) → %s\n",
            new_pos, double(new_pos) / CPR_DEFAULT * 2.0 * M_PI, ok2 ? "成功" : "失败");
        ok = ok1 && ok2;
    } else if (action == "show") {
        // 仅显示，已打印
    } else if (action == "pos") {
        if (!limits_already_invalid &&
            cur_pos <= min_lim && min_lim != INT32_MIN_V) {
            std::fprintf(stderr, "错误: 当前位置 (%d) 已小于等于负向限位 (%d)，拒绝写入正限位\n",
                cur_pos, min_lim);
            std::fprintf(stderr, "  如需重新标定整个区间，请先运行 `clear`\n");
            return 1;
        }
        ok = writeLimit(drv, 0x02, cur_pos);
        std::printf("写入正向限位: %d counts → %s\n",
            cur_pos, ok ? "成功" : "失败");
    } else if (action == "neg") {
        if (!limits_already_invalid &&
            cur_pos >= max_lim && max_lim != INT32_MAX_V) {
            std::fprintf(stderr, "错误: 当前位置 (%d) 已大于等于正向限位 (%d)，拒绝写入负限位\n",
                cur_pos, max_lim);
            std::fprintf(stderr, "  如需重新标定整个区间，请先运行 `clear`\n");
            return 1;
        }
        ok = writeLimit(drv, 0x01, cur_pos);
        std::printf("写入负向限位: %d counts → %s\n",
            cur_pos, ok ? "成功" : "失败");
    } else if (action == "clear") {
        bool ok1 = writeLimit(drv, 0x01, INT32_MIN_V);
        bool ok2 = writeLimit(drv, 0x02, INT32_MAX_V);
        ok = ok1 && ok2;
        std::printf("恢复默认: %s（限位监控已关闭）\n", ok ? "成功" : "失败");
    } else {
        std::fprintf(stderr, "未知操作: %s （应为 pos/neg/show/clear）\n", action.c_str());
        return 1;
    }

    if (!ok) {
        std::fprintf(stderr, "\n✗ SDO 写入失败，请检查驱动器状态\n");
        return 1;
    }

    // 写入后回读确认
    if (action != "show") {
        std::this_thread::sleep_for(100ms);
        readLimit(drv, 0x01, min_lim);
        readLimit(drv, 0x02, max_lim);
        std::printf("\n更新后限位：\n");
        printLimit("负向限位 (607D:01)", min_lim, INT32_MIN_V);
        printLimit("正向限位 (607D:02)", max_lim, INT32_MAX_V);
        std::printf("\n");

        saveToEEPROM(drv);  // 尝试标准保存，RB200-CA 固件会 ACK 但不实际写 EEPROM
        std::printf("⚠ 限位值仅在 RAM 中，断电会丢失。\n");
        std::printf("  请用 VCSDSoft_L 软件「参数管理 → 保存到 EEPROM」固化。\n\n");
    }
    return 0;
}
