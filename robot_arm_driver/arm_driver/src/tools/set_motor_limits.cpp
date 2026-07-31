/**
 * @file set_motor_limits.cpp
 * @brief RB200-CA 电机限制参数查看/设定工具（rad 单位输入，自动换算 pp）
 *
 * 整合旧栈 set_position_limit（软限位）+ arm_hw.yaml 的 max_velocity/pp_accel/
 * pp_decel（速度/加速度上限）为一个工具。不带写选项时只读显示当前值
 * （pp 与 rad 双单位），带选项时写入，--save 固化到 EEPROM。
 *
 * 涉及对象（指令单位 pp = rad × 524288/2π，电子齿轮 1:1）：
 *   607Fh 最大轮廓速度   6081h PP轮廓速度   6083h/6084h 轮廓加/减速度
 *   6085h 快停减速度     607Dh:01/02 软限位  6072h 最大转矩(0.1%)
 *
 * ⚠️ 与 bus.yml 的关系：bus.yml SDO 段在每次起栈时会重写 6081/6083/6084/6085
 *    ——用本工具改这几项只是临时生效，要长期生效请同步改 bus.yml（rad→pp
 *    换算值本工具会打印出来，直接抄进去即可）。软限位 607D 不在 bus.yml 里，
 *    本工具 + --save 即为正式配置方式。
 *
 * 独立工具，直接走 SocketCAN；只做 SDO 读写、不动控制字，但仍建议停栈后使用
 * （避免与主站 SDO 客户端并发）。
 *
 * 用法：
 *   ros2 run robot_arm_driver set_motor_limits <can接口> <node_id> [选项]
 *   无选项           → 只读显示当前限制
 *   --vel    <rad/s> → 6081 PP 轮廓速度
 *   --maxvel <rad/s> → 607F 最大轮廓速度
 *   --acc    <rad/s²>→ 6083 轮廓加速度
 *   --dec    <rad/s²>→ 6084 轮廓减速度
 *   --qstop  <rad/s²>→ 6085 快停减速度
 *   --min    <rad>   → 607D:01 软限位下限
 *   --max    <rad>   → 607D:02 软限位上限
 *   --torque <%>     → 6072 最大转矩（额定的百分比，如 300 = 300.0%）
 *   --save           → 写完固化 EEPROM（1010:02h "save"）
 *   --master <id> --timeout <ms>
 *
 * 例：
 *   ros2 run robot_arm_driver set_motor_limits can0 1                      # 查看
 *   ros2 run robot_arm_driver set_motor_limits can0 1 --vel 1.5 --acc 2.0  # 临时改
 *   ros2 run robot_arm_driver set_motor_limits can0 2 --min -0.981 --max 2.959 --save  # 软限位并固化
 *
 * @date 2026-07-08
 * @copyright Copyright (c) 2026 EMEET
 */

#include "robot_arm_driver/sdo_client.hpp"

#include <cstdlib>
#include <optional>
#include <vector>

using arm_tools::SdoClient;
using arm_tools::rb200;

struct Options {
    std::string ifname;
    int node = -1;
    std::optional<double> vel, maxvel, acc, dec, qstop, min, max, torque;
    bool save = false;
    int master = 127, timeout_ms = 500;
};

static void usage(const char * prog)
{
    std::fprintf(stderr,
        "用法: %s <can接口> <node_id> [选项]\n"
        "  无选项            只读显示当前限制（pp 与 rad 双单位）\n"
        "  --vel    <rad/s>  6081 PP 轮廓速度\n"
        "  --maxvel <rad/s>  607F 最大轮廓速度\n"
        "  --acc    <rad/s2> 6083 轮廓加速度\n"
        "  --dec    <rad/s2> 6084 轮廓减速度\n"
        "  --qstop  <rad/s2> 6085 快停减速度\n"
        "  --min    <rad>    607D:01 软限位下限\n"
        "  --max    <rad>    607D:02 软限位上限\n"
        "  --torque <%%>      6072 最大转矩（如 300 = 300.0%%）\n"
        "  --save            写完固化 EEPROM\n"
        "  --master <id=127> --timeout <ms=500>\n"
        "注意: 6081/6083/6084/6085 每次起栈会被 bus.yml 重写，长期生效请同步改 bus.yml。\n",
        prog);
}

static bool parseArgs(int argc, char * argv[], Options & o)
{
    if (argc < 3) return false;
    o.ifname = argv[1];
    o.node   = std::atoi(argv[2]);
    if (o.node < 1 || o.node > 127) return false;

    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](std::optional<double> & dst) {
            if (i + 1 >= argc) { usage(argv[0]); std::exit(1); }
            dst = std::atof(argv[++i]);
        };
        if      (a == "--vel")     next(o.vel);
        else if (a == "--maxvel")  next(o.maxvel);
        else if (a == "--acc")     next(o.acc);
        else if (a == "--dec")     next(o.dec);
        else if (a == "--qstop")   next(o.qstop);
        else if (a == "--min")     next(o.min);
        else if (a == "--max")     next(o.max);
        else if (a == "--torque")  next(o.torque);
        else if (a == "--save")    o.save = true;
        else if (a == "--master"  && i + 1 < argc) o.master = std::atoi(argv[++i]);
        else if (a == "--timeout" && i + 1 < argc) o.timeout_ms = std::atoi(argv[++i]);
        else { std::fprintf(stderr, "未知选项: %s\n", a.c_str()); return false; }
    }
    return true;
}

static void show(SdoClient & bus)
{
    struct Row { const char * name; uint16_t idx; uint8_t sub; const char * unit; bool sgn; };
    static constexpr Row ROWS[] = {
        {"607F 最大轮廓速度", 0x607F, 0, "rad/s",  false},
        {"6081 PP 轮廓速度 ", 0x6081, 0, "rad/s",  false},
        {"6083 轮廓加速度  ", 0x6083, 0, "rad/s²", false},
        {"6084 轮廓减速度  ", 0x6084, 0, "rad/s²", false},
        {"6085 快停减速度  ", 0x6085, 0, "rad/s²", false},
        {"607D:01 软限位下限", 0x607D, 1, "rad",   true},
        {"607D:02 软限位上限", 0x607D, 2, "rad",   true},
    };

    std::printf("  ── 当前限制（pp ↔ rad，换算系数 524288/2π = 83443.03）──\n");
    for (const auto & r : ROWS) {
        if (r.sgn) {
            int32_t v = bus.sdoReadI32(r.idx, r.sub);
            std::printf("  %s : %11d pp  = %+10.4f %s\n", r.name, v, rb200().ppToRad(v), r.unit);
        } else {
            uint32_t v = bus.sdoRead(r.idx, r.sub);
            std::printf("  %s : %11u pp  = %10.4f %s\n", r.name, v, rb200().ppToRad(static_cast<int32_t>(v)), r.unit);
        }
    }
    uint16_t tq = static_cast<uint16_t>(bus.sdoRead(0x6072u, 0));
    std::printf("  6072 最大转矩     : %11u     = %10.1f %% 额定\n", tq, tq / 10.0);
    int32_t pos = bus.sdoReadI32(0x6064u, 0);
    std::printf("  6064 当前位置     : %11d pp  = %+10.4f rad\n", pos, rb200().ppToRad(pos));
}

int main(int argc, char * argv[])
{
    Options o;
    if (!parseArgs(argc, argv, o)) { usage(argv[0]); return 1; }

    std::printf("\n=== RB200-CA 电机限制参数（节点 %d @ %s）===\n\n", o.node, o.ifname.c_str());

    try {
        SdoClient bus(o.ifname, static_cast<uint8_t>(o.node),
                      static_cast<uint8_t>(o.master), o.timeout_ms);

        // ── 写入（给了哪项写哪项，打印 rad→pp 换算便于回填 bus.yml）──────────
        struct W { const char * name; std::optional<double> v; uint16_t idx; uint8_t sub; bool sgn; };
        const std::vector<W> writes = {
            {"6081 PP 轮廓速度",  o.vel,    0x6081, 0, false},
            {"607F 最大轮廓速度", o.maxvel, 0x607F, 0, false},
            {"6083 轮廓加速度",   o.acc,    0x6083, 0, false},
            {"6084 轮廓减速度",   o.dec,    0x6084, 0, false},
            {"6085 快停减速度",   o.qstop,  0x6085, 0, false},
            {"607D:01 软限位下限", o.min,   0x607D, 1, true},
            {"607D:02 软限位上限", o.max,   0x607D, 2, true},
        };
        bool wrote = false;
        for (const auto & w : writes) {
            if (!w.v) continue;
            int64_t pp;
            if (w.sgn) {                       // 位置类（软限位，可为负）
                pp = rb200().radToPP(*w.v);
            } else {                           // 速度/加速度类（无符号）
                if (*w.v < 0) { std::fprintf(stderr, "错误：%s 不能为负\n", w.name); return 1; }
                pp = rb200().radToVelPP(*w.v);
            }
            bus.sdoWrite(w.idx, w.sub, static_cast<uint32_t>(pp), 4);
            std::printf("  [ OK ] %s ← %.4f rad = %lld pp\n", w.name, *w.v, static_cast<long long>(pp));
            wrote = true;
        }
        if (o.torque) {
            auto t = static_cast<uint32_t>(*o.torque * 10.0 + 0.5);   // % → 0.1%
            bus.sdoWrite(0x6072u, 0, t, 2);
            std::printf("  [ OK ] 6072 最大转矩 ← %.1f %% (=%u)\n", *o.torque, t);
            wrote = true;
        }
        if (o.save) {
            bus.saveToEeprom();
            std::printf("  [ OK ] 已固化 EEPROM（1010:02h \"save\"）\n");
        }
        if (wrote) {
            std::printf("\n  ⚠️ 6081/6083/6084/6085 每次起栈会被 bus.yml SDO 段重写；\n"
                        "     要长期生效请把上面的 pp 值同步进 bus.yml。\n\n");
        }

        show(bus);
        std::printf("\n");
    } catch (const std::exception & e) {
        std::fprintf(stderr, "错误：%s\n", e.what());
        return 1;
    }
    return 0;
}
