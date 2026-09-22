/**
 * @file sdo_client.hpp
 * @brief 极简 CANopen SDO 客户端（SocketCAN，expedited ≤4 字节）+ 主站心跳 + 单位换算
 *
 * 供独立调试/标定工具复用（set_encoder_zero / set_motor_limits）。
 * ⚠️ 这些工具直接占用总线控制字，运行前必须先停掉 ros2_control 栈。
 *
 * 单位换算：统一走 motor_unit_converter.hpp（MotorUnitConverter，单一权威），
 * 本文件仅提供 RB200-CA 默认参数的共享实例 rb200()。
 *
 * @date 2026-07-08
 * @copyright Copyright (c) 2026 EMEET
 */

#pragma once

#include "robot_arm_driver/motor_unit_converter.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

namespace arm_tools {

using namespace std::chrono_literals;
using clk = std::chrono::steady_clock;

/** RB200-CA 共享换算器（524288 counts/rev，与 bus.yml scale 同源） */
inline const arm::MotorUnitConverter & rb200()
{
    static const arm::MotorUnitConverter conv(524288);
    return conv;
}

class SdoClient {
public:
    SdoClient(const std::string & ifname, uint8_t node_id, uint8_t master_id, int timeout_ms)
        : node_(node_id), master_(master_id), timeout_ms_(timeout_ms)
    {
        fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) throw std::runtime_error("socket(PF_CAN) 失败");

        // 只收本节点的 SDO 响应，其他流量（心跳/EMCY/PDO）内核层过滤掉
        can_filter flt{};
        flt.can_id   = 0x580u + node_;
        flt.can_mask = CAN_SFF_MASK;
        ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt));

        timeval tv{0, 50 * 1000};  // recv 分片超时 50ms，总超时在 recvResp 里控制
        ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        ifreq ifr{};
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
            throw std::runtime_error("找不到 CAN 接口 " + ifname);
        sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
            throw std::runtime_error("bind " + ifname + " 失败");
    }

    ~SdoClient() { if (fd_ >= 0) ::close(fd_); }
    SdoClient(const SdoClient &) = delete;
    SdoClient & operator=(const SdoClient &) = delete;

    /** NMT 命令（0x000 广播帧，只发不收）：0x01 Start / 0x02 Stop / 0x80 Pre-Op /
     *  0x81 Reset Node / 0x82 Reset Comm。目标为本客户端绑定的节点。 */
    void nmt(uint8_t cs)
    {
        can_frame f{};
        f.can_id  = 0x000u;
        f.can_dlc = 2;
        f.data[0] = cs;
        f.data[1] = node_;
        (void)::write(fd_, &f, sizeof(f));
    }

    /** 主站心跳（Operational）。RB200 心跳超时会报 ER.E20/E21，长流程中要持续发。 */
    void heartbeat()
    {
        can_frame f{};
        f.can_id  = 0x700u + master_;
        f.can_dlc = 1;
        f.data[0] = 0x05u;
        (void)::write(fd_, &f, sizeof(f));
    }

    void sdoWrite(uint16_t index, uint8_t sub, uint32_t value, int size)
    {
        static constexpr uint8_t CS[] = {0, 0x2F, 0x2B, 0x27, 0x23};
        can_frame req{};
        req.can_id  = 0x600u + node_;
        req.can_dlc = 8;
        req.data[0] = CS[size];
        req.data[1] = index & 0xFF;
        req.data[2] = index >> 8;
        req.data[3] = sub;
        std::memcpy(&req.data[4], &value, 4);
        if (::write(fd_, &req, sizeof(req)) != sizeof(req))
            throw std::runtime_error("CAN 发送失败");

        can_frame resp = recvResp(index, sub, "写");
        if (resp.data[0] != 0x60u)
            throw std::runtime_error(fmtAbort(resp, index, sub, "写"));
    }

    uint32_t sdoRead(uint16_t index, uint8_t sub)
    {
        can_frame req{};
        req.can_id  = 0x600u + node_;
        req.can_dlc = 8;
        req.data[0] = 0x40u;
        req.data[1] = index & 0xFF;
        req.data[2] = index >> 8;
        req.data[3] = sub;
        if (::write(fd_, &req, sizeof(req)) != sizeof(req))
            throw std::runtime_error("CAN 发送失败");

        can_frame resp = recvResp(index, sub, "读");
        if ((resp.data[0] & 0xE0u) != 0x40u)
            throw std::runtime_error(fmtAbort(resp, index, sub, "读"));
        uint32_t v = 0;
        std::memcpy(&v, &resp.data[4], 4);
        return v;
    }

    int32_t  sdoReadI32(uint16_t i, uint8_t s) { return static_cast<int32_t>(sdoRead(i, s)); }
    uint16_t statusWord() { return static_cast<uint16_t>(sdoRead(0x6041u, 0)); }

    /**
     * 参数固化到 EEPROM：1010:01h 写 ASCII "save"（0x65766173）。
     *
     * ⚠️ EDS 与《RB200-CA 使用说明书（简版）V1.0》都写的是 1010:02h，但实机固件
     *    只实现 sub1——读 1010:00 返回 1（最大子索引=1），写 1010:02 直接
     *    SDO Abort 0x06090011（子索引不存在），即固化静默失效、断电即丢。
     *    2026-09-20 在过渡板 can1 节点 1 上实测确认。
     */
    void saveToEeprom() { sdoWrite(0x1010u, 0x01, 0x65766173u, 4); }

    /** 轮询状态字直至 (sw & mask) == value；期间持续发主站心跳。 */
    uint16_t waitStatus(uint16_t mask, uint16_t value, int timeout_ms, const char * desc)
    {
        auto deadline = clk::now() + std::chrono::milliseconds(timeout_ms);
        uint16_t sw = 0;
        while (clk::now() < deadline) {
            heartbeat();
            sw = statusWord();
            if ((sw & mask) == value) return sw;
            std::this_thread::sleep_for(50ms);
        }
        throw std::runtime_error(std::string(desc) + " 超时（状态字 0x" + hex16(sw) + "）");
    }

    /**
     * 下发控制字并等待状态迁移；等待期间重复写同一控制字。
     * （部分从站——含 canopen_fake_slaves——只在 6040 被写入时评估状态机，
     *  真栈里主站 RPDO 100Hz 重复写掩盖了这一点；重复写对真机同样无害。）
     */
    uint16_t commandState(uint16_t cw, uint16_t mask, uint16_t value,
                          int timeout_ms, const char * desc)
    {
        auto deadline = clk::now() + std::chrono::milliseconds(timeout_ms);
        uint16_t sw = 0;
        while (clk::now() < deadline) {
            heartbeat();
            sdoWrite(0x6040u, 0, cw, 2);
            sw = statusWord();
            if ((sw & mask) == value) return sw;
            std::this_thread::sleep_for(50ms);
        }
        throw std::runtime_error(std::string(desc) + " 超时（状态字 0x" + hex16(sw) + "）");
    }

private:
    can_frame recvResp(uint16_t index, uint8_t sub, const char * op)
    {
        auto deadline = clk::now() + std::chrono::milliseconds(timeout_ms_);
        can_frame resp{};
        while (clk::now() < deadline) {
            ssize_t n = ::read(fd_, &resp, sizeof(resp));
            if (n == sizeof(resp)) return resp;  // 内核已过滤，只会是 0x580+node
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf), "SDO %s %04X:%02X 无响应（超时 %dms），检查接线/节点号/供电",
                      op, index, sub, timeout_ms_);
        throw std::runtime_error(buf);
    }

    static std::string fmtAbort(const can_frame & resp, uint16_t index, uint8_t sub, const char * op)
    {
        if (resp.data[0] == 0x80u) {
            uint32_t abort = 0;
            std::memcpy(&abort, &resp.data[4], 4);
            char buf[96];
            std::snprintf(buf, sizeof(buf), "SDO Abort 0x%08X %s %04X:%02X", abort, op, index, sub);
            return buf;
        }
        char buf[96];
        std::snprintf(buf, sizeof(buf), "SDO %s %04X:%02X 响应异常 cs=0x%02X", op, index, sub, resp.data[0]);
        return buf;
    }

    static std::string hex16(uint16_t v)
    {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%04X", v);
        return buf;
    }

    int     fd_ = -1;
    uint8_t node_;
    uint8_t master_;
    int     timeout_ms_;
};

}  // namespace arm_tools
