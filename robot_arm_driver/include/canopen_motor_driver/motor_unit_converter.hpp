/**
 * @file motor_unit_converter.hpp
 * @brief 电机单位换算器：SI 单位 ↔ 驱动器指令单位（pp）
 *
 * 换算核心参数：counts_per_rev（每圈输出轴对应的指令单位数）
 *
 * 推算方法（RB200-CA 实测校准，型号 D-JM，减速比 100:1）：
 *   0x6091 = 1/1，0x6064 = 减速机端编码器（JM，19-bit = 524288 counts/rev）
 *   0x6063 = 电机端编码器（D，17-bit = 131072 counts/rev）
 *   比值验证：Δ6063/Δ6064 ≈ 131072×100/524288 = 25 ✓（位置移动实测确认）
 *   → counts_per_rev = 2^19 = 524288
 *
 * 换算关系：
 *   position_rad  = position_pp  × 2π / counts_per_rev
 *   velocity_rad_s = velocity_pp_s × 2π / counts_per_rev
 *   accel_rad_s2  = accel_pp_s2  × 2π / counts_per_rev
 *
 * 所属模块：hardware_driver/include/canopen_motor_driver/
 *
 * @version 1.1
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 EMEET
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace arm {

class MotorUnitConverter {
public:
    /**
     * @brief 构造换算器
     * @param counts_per_rev 每圈输出轴对应的指令单位数，必须大于 0
     */
    explicit MotorUnitConverter(int64_t counts_per_rev)
        : cpr_(counts_per_rev)
    {
        if (cpr_ <= 0) {
            throw std::invalid_argument("counts_per_rev must be > 0");
        }
    }

    int64_t countsPerRev() const { return cpr_; }

    // ── SI → 指令单位 ──────────────────────────────────────────────────────

    /** rad → pp（位置） */
    int32_t radToPP(double rad) const {
        return static_cast<int32_t>(rad * cpr_ / TWO_PI);
    }

    /** rad/s → pp/s（速度，无符号，用于 profile 参数） */
    uint32_t radToVelPP(double rad_s) const {
        return static_cast<uint32_t>(std::abs(rad_s) * cpr_ / TWO_PI);
    }

    /** rad/s → pp/s（速度，有符号，用于速度目标指令） */
    int32_t radToVelPPSigned(double rad_s) const {
        return static_cast<int32_t>(rad_s * cpr_ / TWO_PI);
    }

    /** rad/s² → pp/s²（加减速） */
    uint32_t radToAccPP(double rad_s2) const {
        return static_cast<uint32_t>(std::abs(rad_s2) * cpr_ / TWO_PI);
    }

    // ── 指令单位 → SI ──────────────────────────────────────────────────────

    /** pp → rad（位置） */
    double ppToRad(int32_t pp) const {
        return pp * TWO_PI / cpr_;
    }

    /** pp/s → rad/s（速度，与 ppToRad 换算因子相同） */
    double ppToRadS(int32_t pp_s) const {
        return ppToRad(pp_s);
    }

private:
    static constexpr double TWO_PI = 2.0 * M_PI;
    int64_t cpr_;
};

} // namespace arm
