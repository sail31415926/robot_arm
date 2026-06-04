/**
 * @file canopen_motor_driver.cpp
 * @brief CiA402 CANopen 电机驱动实现（SocketCAN）
 *
 * 通用 DS402 驱动，适用于任何实现 CiA402 标准的 CANopen 伺服/步进电机。
 * 使用 Linux 标准 SocketCAN 接口，无需第三方 CAN 库。
 *
 * 实现内容：
 *   - SocketCAN 套接字的打开、绑定与关闭
 *   - SDO expedited 阻塞式读写（CiA301）
 *   - NMT 网络管理命令
 *   - DS402 伺服状态机（使能 / 禁用 / 故障复位）
 *   - 轮廓位置模式 PP（1）、轮廓速度模式 PV（3）
 *   - 轮廓力矩模式 PT（4）
 *   - 插补位置模式 IP（7）：自动配置 RPDO1 映射，位置点通过 PDO 帧触发执行周期
 *   - 回零模式 HM（6），支持等待期间心跳回调
 *   - 位置 / 速度 / 力矩 / 状态字反馈读取
 *
 * 所属模块：hardware_driver/src/
 *
 * @version 1.1
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 EMEET
 */

#include "canopen_motor_driver/canopen_motor_driver.hpp"

#include <cerrno>
#include <cstring>
#include <chrono>
#include <thread>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace arm {

static constexpr uint32_t SDO_RX = 0x600u;
static constexpr uint32_t SDO_TX = 0x580u;
static constexpr uint32_t NMT_ID = 0x000u;

// ─── 构造 / 析构 ──────────────────────────────────────────────────────────────

CanopenMotorDriver::CanopenMotorDriver(const std::string& ifname,
                                       uint8_t node_id,
                                       int sdo_timeout_ms)
    : ifname_(ifname), node_id_(node_id), sdo_timeout_ms_(sdo_timeout_ms) {}

CanopenMotorDriver::~CanopenMotorDriver() { close(); }

// ─── 打开 / 关闭 ─────────────────────────────────────────────────────────────

bool CanopenMotorDriver::init() {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) return false;

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd_); fd_ = -1; return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd_); fd_ = -1; return false;
    }
    return true;
}

void CanopenMotorDriver::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// ─── 内部：CAN 帧收发 ────────────────────────────────────────────────────────

static bool sendFrame(int fd, uint32_t cob_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame{};
    frame.can_id  = cob_id & CAN_SFF_MASK;
    frame.can_dlc = dlc;
    if (dlc > 0) std::memcpy(frame.data, data, dlc);
    return ::write(fd, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
}

static bool recvFrame(int fd, struct can_frame& frame, int timeout_ms) {
    using clk = std::chrono::steady_clock;
    auto deadline = clk::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
        auto rem = std::chrono::duration_cast<std::chrono::microseconds>(
            deadline - clk::now()).count();
        if (rem <= 0) return false;
        fd_set rdset; FD_ZERO(&rdset); FD_SET(fd, &rdset);
        struct timeval tv{ rem / 1000000, rem % 1000000 };
        int ret = ::select(fd + 1, &rdset, nullptr, nullptr, &tv);
        if (ret < 0 && errno == EINTR) continue;
        if (ret <= 0) return false;
        if (::read(fd, &frame, sizeof(struct can_frame)) ==
            static_cast<ssize_t>(sizeof(struct can_frame))) return true;
    }
}

// ─── NMT ─────────────────────────────────────────────────────────────────────

bool CanopenMotorDriver::nmtCommand(uint8_t cmd) {
    uint8_t data[2] = { cmd, node_id_ };
    return sendFrame(fd_, NMT_ID, data, 2);
}

bool CanopenMotorDriver::saveParameters(int timeout_ms) {
    if (fd_ < 0) return false;
    // DS301: write ASCII "save" (little-endian: 0x73 0x61 0x76 0x65) to 0x1010:02
    uint8_t req[8]{};
    req[0] = 0x23u;          // CS: expedited download, 4 bytes
    req[1] = 0x10u; req[2] = 0x10u;  // index 0x1010h (little-endian)
    req[3] = 0x02u;          // subindex 02h: save all parameters
    req[4] = 0x73u; req[5] = 0x61u;  // 's', 'a'
    req[6] = 0x76u; req[7] = 0x65u;  // 'v', 'e'
    if (!sendFrame(fd_, SDO_RX + node_id_, req, 8)) return false;

    struct can_frame resp{};
    using clk = std::chrono::steady_clock;
    auto deadline = clk::now() + std::chrono::milliseconds(timeout_ms);
    while (clk::now() < deadline) {
        int rem = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - clk::now()).count());
        if (!recvFrame(fd_, resp, rem)) {
            std::fprintf(stderr, "  [saveParameters] SDO 无响应（超时 %dms），驱动器未实现 0x1010h 或不在正确状态\n",
                timeout_ms);
            return false;
        }
        if ((resp.can_id & CAN_SFF_MASK) == SDO_TX + node_id_) {
            if (resp.data[0] == 0x60u) return true;
            if (resp.data[0] == 0x80u) {
                uint32_t abort = uint32_t(resp.data[4])
                               | uint32_t(resp.data[5]) << 8
                               | uint32_t(resp.data[6]) << 16
                               | uint32_t(resp.data[7]) << 24;
                std::fprintf(stderr, "  [saveParameters] SDO Abort 0x%08X — ", abort);
                switch (abort) {
                    case 0x06010000u: std::fprintf(stderr, "不支持该对象访问\n"); break;
                    case 0x06090011u: std::fprintf(stderr, "子索引不存在\n"); break;
                    case 0x08000020u: std::fprintf(stderr, "数据无法传输（应用层拒绝）\n"); break;
                    case 0x08000021u: std::fprintf(stderr, "数据无法传输（本地控制拒绝）\n"); break;
                    case 0x08000022u: std::fprintf(stderr, "当前设备状态不允许\n"); break;
                    default:          std::fprintf(stderr, "未知原因\n"); break;
                }
            }
            return false;
        }
    }
    return false;
}

// ─── SDO ─────────────────────────────────────────────────────────────────────

bool CanopenMotorDriver::writeSDO(uint16_t index, uint8_t subindex,
                                   const void* data, uint8_t len) {
    if (fd_ < 0 || len == 0 || len > 4) return false;
    static const uint8_t CS[] = { 0, 0x2F, 0x2B, 0x27, 0x23 };
    uint8_t req[8]{};
    req[0] = CS[len];
    req[1] = index & 0xFFu;  req[2] = (index >> 8) & 0xFFu;
    req[3] = subindex;
    std::memcpy(&req[4], data, len);
    if (!sendFrame(fd_, SDO_RX + node_id_, req, 8)) return false;

    struct can_frame resp{};
    using clk = std::chrono::steady_clock;
    auto deadline = clk::now() + std::chrono::milliseconds(sdo_timeout_ms_);
    while (clk::now() < deadline) {
        int rem = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - clk::now()).count());
        if (!recvFrame(fd_, resp, rem)) break;
        if ((resp.can_id & CAN_SFF_MASK) == SDO_TX + node_id_)
            return resp.data[0] == 0x60u;
    }
    return false;
}

bool CanopenMotorDriver::readSDO(uint16_t index, uint8_t subindex,
                                  void* data, uint8_t& len) {
    if (fd_ < 0) return false;
    uint8_t req[8]{};
    req[0] = 0x40u;
    req[1] = index & 0xFFu;  req[2] = (index >> 8) & 0xFFu;
    req[3] = subindex;
    if (!sendFrame(fd_, SDO_RX + node_id_, req, 8)) return false;

    struct can_frame resp{};
    using clk = std::chrono::steady_clock;
    auto deadline = clk::now() + std::chrono::milliseconds(sdo_timeout_ms_);
    while (clk::now() < deadline) {
        int rem = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - clk::now()).count());
        if (!recvFrame(fd_, resp, rem)) break;
        if ((resp.can_id & CAN_SFF_MASK) == SDO_TX + node_id_) {
            uint8_t cs = resp.data[0];
            if (cs == 0x80u) return false;
            if (cs & 0x02u) {
                len = (cs & 0x01u) ? (4u - ((cs >> 2u) & 0x03u)) : 4u;
                std::memcpy(data, &resp.data[4], len);
                return true;
            }
        }
    }
    return false;
}

// ─── SDO 类型化辅助 ──────────────────────────────────────────────────────────

bool CanopenMotorDriver::writeSDOu8(uint16_t i, uint8_t s, uint8_t v) {
    return writeSDO(i, s, &v, 1); }
bool CanopenMotorDriver::writeSDOu16(uint16_t i, uint8_t s, uint16_t v) {
    uint8_t b[2] = { uint8_t(v), uint8_t(v>>8) }; return writeSDO(i, s, b, 2); }
bool CanopenMotorDriver::writeSDOu32(uint16_t i, uint8_t s, uint32_t v) {
    uint8_t b[4] = { uint8_t(v),uint8_t(v>>8),uint8_t(v>>16),uint8_t(v>>24) };
    return writeSDO(i, s, b, 4); }
bool CanopenMotorDriver::writeSDOi32(uint16_t i, uint8_t s, int32_t v) {
    return writeSDOu32(i, s, static_cast<uint32_t>(v)); }
bool CanopenMotorDriver::readSDOu16(uint16_t i, uint8_t s, uint16_t& out) {
    uint8_t b[4]{}; uint8_t len = 0;
    if (!readSDO(i, s, b, len)) return false;
    out = uint16_t(b[0]) | uint16_t(uint16_t(b[1]) << 8); return true; }
bool CanopenMotorDriver::readSDOi32(uint16_t i, uint8_t s, int32_t& out) {
    uint8_t b[4]{}; uint8_t len = 0;
    if (!readSDO(i, s, b, len)) return false;
    out = int32_t(uint32_t(b[0])|(uint32_t(b[1])<<8)|
                  (uint32_t(b[2])<<16)|(uint32_t(b[3])<<24));
    return true; }

// ─── 状态字轮询 ───────────────────────────────────────────────────────────────

bool CanopenMotorDriver::waitStatusWord(uint16_t mask, uint16_t expected, int timeout_ms) {
    using clk = std::chrono::steady_clock;
    auto deadline = clk::now() + std::chrono::milliseconds(timeout_ms);
    while (clk::now() < deadline) {
        uint16_t sw = 0;
        if (readSDOu16(0x6041u, 0, sw) && (sw & mask) == expected) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

// ─── DS402 状态机 ─────────────────────────────────────────────────────────────

bool CanopenMotorDriver::resetFault() {
    return writeSDOu16(0x6040u, 0, 0x0080u) && writeSDOu16(0x6040u, 0, 0x0000u);
}

bool CanopenMotorDriver::enable(int timeout_ms) {
    if (!nmtCommand(0x01)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    uint16_t sw = 0;
    if (readSDOu16(0x6041u, 0, sw) && (sw & 0x0008u)) {
        resetFault();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const int step = timeout_ms / 3;
    if (!writeSDOu16(0x6040u, 0, 0x0006u)) return false;
    if (!waitStatusWord(0x006Fu, 0x0021u, step)) return false;
    if (!writeSDOu16(0x6040u, 0, 0x0007u)) return false;
    if (!waitStatusWord(0x006Fu, 0x0023u, step)) return false;
    if (!writeSDOu16(0x6040u, 0, 0x000Fu)) return false;
    if (!waitStatusWord(0x006Fu, 0x0027u, step)) return false;
    return true;
}

bool CanopenMotorDriver::disable() {
    return writeSDOu16(0x6040u, 0, 0x0006u);
}

// ─── PP / PV 模式 ────────────────────────────────────────────────────────────

bool CanopenMotorDriver::setProfilePositionMode(uint32_t vel, uint32_t accel, uint32_t decel) {
    return writeSDOu8 (0x6060u, 0, 0x01u)  &&
           writeSDOu32(0x6081u, 0, vel)     &&
           writeSDOu32(0x6083u, 0, accel)   &&
           writeSDOu32(0x6084u, 0, decel);
}

bool CanopenMotorDriver::setProfileVelocity(uint32_t vel) {
    return writeSDOu32(0x6081u, 0, vel);
}

bool CanopenMotorDriver::moveToPosition(int32_t pos, bool relative,
                                         bool wait_done, int timeout_ms) {
    if (!writeSDOi32(0x607Au, 0, pos)) return false;
    uint16_t cw = 0x003Fu;
    if (relative) cw |= 0x0040u;
    if (!writeSDOu16(0x6040u, 0, cw)) return false;
    if (!waitStatusWord(0x1000u, 0x1000u, 500)) return false;
    cw &= ~0x0010u;
    if (!writeSDOu16(0x6040u, 0, cw)) return false;
    return wait_done ? waitStatusWord(0x0400u, 0x0400u, timeout_ms) : true;
}

bool CanopenMotorDriver::setProfileVelocityMode(uint32_t accel, uint32_t decel) {
    return writeSDOu8 (0x6060u, 0, 0x03u) &&
           writeSDOu32(0x6083u, 0, accel)  &&
           writeSDOu32(0x6084u, 0, decel);
}

bool CanopenMotorDriver::setTargetVelocity(int32_t vel_pps) {
    return writeSDOi32(0x60FFu, 0, vel_pps);
}

// ─── 轮廓力矩模式 4-PT ───────────────────────────────────────────────────────

bool CanopenMotorDriver::setProfileTorqueMode(uint16_t max_torque_permille,
                                               uint32_t torque_slope_ms,
                                               uint32_t max_vel) {
    return writeSDOu8 (0x6060u, 0, 0x04u)              &&
           writeSDOu16(0x6072u, 0, max_torque_permille) &&
           writeSDOu32(0x6087u, 0, torque_slope_ms)     &&
           writeSDOu32(0x607Fu, 0, max_vel);            // 速度上限，PT 模式必须 > 0
}

bool CanopenMotorDriver::setTargetTorque(int16_t torque_permille) {
    return writeSDOu16(0x6071u, 0, static_cast<uint16_t>(torque_permille));
}

// ─── 插补位置模式 7-IP ───────────────────────────────────────────────────────

bool CanopenMotorDriver::setInterpolatedPositionMode(uint8_t period_ms,
                                                      uint32_t max_vel) {
    if (period_ms < 1) period_ms = 1;

    // ── 按手册 3.5 节表格的顺序执行 ──────────────────────────────────────────

    // Step 0-1: 插补周期
    if (!writeSDOu8(0x60C2u, 1, period_ms)) return false;
    if (!writeSDOu8(0x60C2u, 2, static_cast<uint8_t>(-3))) return false;

    // Step 2: 读当前位置写入初始插补点
    int32_t cur_pos = 0;
    readSDOi32(0x6064u, 0, cur_pos);
    if (!writeSDOi32(0x60C1u, 1, cur_pos)) return false;

    // Step 3: 切换到 IP 模式 + 最大速度
    if (!writeSDOu8(0x6060u, 0, 0x07u)) return false;
    if (!writeSDOu32(0x607Fu, 0, max_vel)) return false;

    // Step 4-6: DS402 状态机全程用 SDO（不依赖 RPDO），避免状态转换时 RPDO 被驱动器重置
    if (!writeSDOu16(0x6040u, 0, 0x0006u)) return false;  // Shutdown
    if (!waitStatusWord(0x006Fu, 0x0021u, 2000)) return false;
    if (!writeSDOu16(0x6040u, 0, 0x0007u)) return false;  // Switch On
    if (!waitStatusWord(0x006Fu, 0x0023u, 2000)) return false;
    if (!writeSDOu16(0x6040u, 0, 0x000Fu)) return false;  // Enable Operation
    if (!waitStatusWord(0x006Fu, 0x0027u, 2000)) return false;

    // Step 7: 在 Operation Enabled 稳定后才配置 RPDO（避免状态转换期间映射被重置）
    if (!configureIPModePDO()) return false;

    // Step 8: 第一帧 PDO 同时完成"使能插补（bit4=1）"和"设定初始位置"
    return sendInterpolationPDO(cur_pos);
}

bool CanopenMotorDriver::setInterpolatedPosition(int32_t pos_pp) {
    // IP 模式必须用 PDO 帧触发驱动器执行周期，不能用 SDO
    return sendInterpolationPDO(pos_pp);
}

bool CanopenMotorDriver::configureIPModePDO() {
    uint32_t cob_id = 0x200u + node_id_;

    // 1. 禁用 RPDO1（bit31=1）
    if (!writeSDOu32(0x1400u, 1, 0x80000000u | cob_id)) return false;

    // 2. 清空映射
    if (!writeSDOu8(0x1600u, 0, 0x00u)) return false;

    // 3. 映射对象 1：0x6040:00（控制字，UINT16 = 16bit = 0x10）
    //    手册要求控制字必须在 RPDO 中，确保 bit4（使能插补）持续有效
    if (!writeSDOu32(0x1600u, 1, 0x60400010u)) return false;

    // 4. 映射对象 2：0x60C1:01（插补位置，INT32 = 32bit = 0x20）
    if (!writeSDOu32(0x1600u, 2, 0x60C10120u)) return false;

    // 5. 使能 2 个映射对象（共 6 字节）
    if (!writeSDOu8(0x1600u, 0, 0x02u)) return false;

    // 6. 先设传输类型（必须在 PDO 禁用状态下），再启用 RPDO1
    if (!writeSDOu8 (0x1400u, 2, 0x01u)) return false;   // 0x01 = 同步型，SYNC 到来时执行
    if (!writeSDOu32(0x1400u, 1, cob_id)) return false;  // 清除 bit31 = 启用

    return true;
}

bool CanopenMotorDriver::sendInterpolationData(int32_t pos_pp) {
    // RPDO1 映射：[6040h(2B) + 60C1h:01(4B)] = 6 字节
    // 控制字持续保持 0x001F（Enable Operation + bit4=1 使能插补）
    constexpr uint16_t cw = 0x001Fu;
    uint8_t data[6] = {
        static_cast<uint8_t>(cw),
        static_cast<uint8_t>(cw >> 8),
        static_cast<uint8_t>(pos_pp),
        static_cast<uint8_t>(pos_pp >> 8),
        static_cast<uint8_t>(pos_pp >> 16),
        static_cast<uint8_t>(pos_pp >> 24)
    };
    return sendFrame(fd_, 0x200u + node_id_, data, 6);
}

bool CanopenMotorDriver::sendSYNC() {
    return sendFrame(fd_, 0x080u, nullptr, 0);
}

bool CanopenMotorDriver::sendInterpolationPDO(int32_t pos_pp) {
    // 单轴场景快捷方法：发 RPDO1 数据 + SYNC 触发执行
    if (!sendInterpolationData(pos_pp)) return false;
    return sendSYNC();
}

// ─── 回零模式 6-HM ───────────────────────────────────────────────────────────

bool CanopenMotorDriver::setHomingMode(uint8_t method, uint32_t fast_vel,
                                        uint32_t slow_vel, uint32_t accel,
                                        int32_t offset) {
    if (slow_vel < 1) slow_vel = 1;
    return writeSDOu8 (0x6060u, 0, 0x06u)       &&
           writeSDOu8 (0x6098u, 0, method)       &&
           writeSDOu32(0x6099u, 1, fast_vel)     &&
           writeSDOu32(0x6099u, 2, slow_vel)     &&
           writeSDOu32(0x609Au, 0, accel)        &&
           writeSDOi32(0x607Cu, 0, offset);
}

bool CanopenMotorDriver::startHoming(int timeout_ms,
                                      std::function<void()> on_heartbeat) {
    // bit4=1：触发回零
    if (!writeSDOu16(0x6040u, 0, 0x001Fu)) return false;

    using clk = std::chrono::steady_clock;
    auto deadline  = clk::now() + std::chrono::milliseconds(timeout_ms);
    auto last_hb   = clk::now();
    bool success = false;
    while (clk::now() < deadline) {
        uint16_t sw = 0;
        if (readSDOu16(0x6041u, 0, sw)) {
            if (sw & 0x2000u) break;                        // bit13：回零错误
            if (sw & 0x1000u) { success = true; break; }   // bit12：回零完成
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // 每 80ms 触发一次心跳，防止单线程执行器阻塞期间驱动器报 0x8130
        if (on_heartbeat) {
            auto now = clk::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_hb).count() >= 80) {
                on_heartbeat();
                last_hb = now;
            }
        }
    }

    writeSDOu16(0x6040u, 0, 0x000Fu);  // bit4=0：清除触发
    return success;
}

// ─── 心跳生产者 ──────────────────────────────────────────────────────────────

bool CanopenMotorDriver::sendHeartbeat(uint8_t master_node_id) {
    uint8_t data = 0x05u;  // NMT state: Operational
    return sendFrame(fd_, 0x700u + master_node_id, &data, 1);
}

// ─── 反馈 ─────────────────────────────────────────────────────────────────────

int32_t  CanopenMotorDriver::getPosition()   { int32_t  v=0; readSDOi32(0x6064u,0,v); return v; }
int32_t  CanopenMotorDriver::getVelocity()   { int32_t  v=0; readSDOi32(0x606Cu,0,v); return v; }
int16_t  CanopenMotorDriver::getTorque()     { uint16_t v=0; readSDOu16(0x6077u,0,v); return static_cast<int16_t>(v); }
uint16_t CanopenMotorDriver::getStatusWord() { uint16_t v=0; readSDOu16(0x6041u,0,v); return v; }

} // namespace arm
