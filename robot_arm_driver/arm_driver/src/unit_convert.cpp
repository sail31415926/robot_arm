/**
 * @file unit_convert.cpp
 * @brief 命令行单位换算器：SI(rad) ↔ RB200-CA 指令单位(pp)
 *
 * 所有手工换算的统一入口（改 bus.yml 限速值、核对 candump 报文等）。
 * 换算全部走 MotorUnitConverter（motor_unit_converter.hpp，单一权威，
 * counts_per_rev=524288，与 bus.yml scale_pos_to_dev=83443.026748 同源）。
 * 位置/速度/加速度换算因子相同（rad、rad/s、rad/s² 通用）。
 *
 * 用法：
 *   ros2 run robot_arm_driver unit_convert rad2pp <值...>   # rad → pp
 *   ros2 run robot_arm_driver unit_convert pp2rad <值...>   # pp → rad
 *   ros2 run robot_arm_driver unit_convert                  # 打印常用速查表
 *
 * 例：
 *   ros2 run robot_arm_driver unit_convert rad2pp 1.5 2.0
 *   ros2 run robot_arm_driver unit_convert pp2rad 125165 8344
 *
 * @date 2026-07-08
 * @copyright Copyright (c) 2026 EMEET
 */

#include "motor_unit_converter.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static const arm::MotorUnitConverter & conv()
{
    static const arm::MotorUnitConverter c(524288);
    return c;
}

static void table()
{
    std::printf("\n=== RB200-CA 单位换算速查（counts_per_rev=524288，因子 524288/2π=83443.03）===\n");
    std::printf("  位置/速度/加速度同因子：rad↔pp、rad/s↔pp/s、rad/s²↔pp/s²\n\n");
    std::printf("  %-12s%-12s        %-12s%-12s\n", "rad", "pp", "pp", "rad");
    const double rads[] = {0.05, 0.1, 0.2, 0.5, 0.8, 1.0, 1.5, 1.5708, 2.0, 2.618, 3.14, 6.2832};
    const int    pps[]  = {1000, 4172, 8344, 41722, 83443, 131072, 262144, 524288, 834430, 1000000, 2097152, 8388608};
    for (size_t i = 0; i < sizeof(rads) / sizeof(rads[0]); ++i) {
        std::printf("  %-12.4f%-12d        %-12d%-12.4f\n",
                    rads[i], conv().radToPP(rads[i]),
                    pps[i], conv().ppToRad(pps[i]));
    }
    std::printf("\n");
}

int main(int argc, char * argv[])
{
    if (argc < 2) { table(); return 0; }

    const bool rad2pp = std::strcmp(argv[1], "rad2pp") == 0;
    const bool pp2rad = std::strcmp(argv[1], "pp2rad") == 0;
    if ((!rad2pp && !pp2rad) || argc < 3) {
        std::fprintf(stderr,
            "用法: %s rad2pp <值...> | pp2rad <值...> | (无参数=速查表)\n", argv[0]);
        return 1;
    }

    for (int i = 2; i < argc; ++i) {
        if (rad2pp) {
            double rad = std::atof(argv[i]);
            std::printf("%12.6f rad  =  %d pp\n", rad, conv().radToPP(rad));
        } else {
            int32_t pp = static_cast<int32_t>(std::atol(argv[i]));
            std::printf("%12d pp  =  %.6f rad\n", pp, conv().ppToRad(pp));
        }
    }
    return 0;
}
