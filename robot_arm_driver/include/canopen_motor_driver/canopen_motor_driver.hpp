/**
 * @file canopen_motor_driver.hpp
 * @brief CiA402 CANopen 电机驱动接口（SocketCAN）
 *
 * 通用 DS402 驱动，适用于任何实现 CiA402 标准的 CANopen 伺服/步进电机。
 * 使用 Linux 标准 SocketCAN，无需第三方 CAN 库。
 *
 * 支持的运动模式：
 *   - PP  轮廓位置模式（1）：setProfilePositionMode() + moveToPosition()
 *   - PV  轮廓速度模式（3）：setProfileVelocityMode() + setTargetVelocity()
 *   - PT  轮廓力矩模式（4）：setProfileTorqueMode()  + setTargetTorque()
 *   - IP  插补位置模式（7）：setInterpolatedPositionMode() + setInterpolatedPosition()
 *              自动配置 RPDO1 映射，位置点通过 PDO 帧发送，触发驱动器插补执行周期
 *   - HM  回零模式    （6）：setHomingMode() + startHoming()
 *
 * 反馈读取：
 *   getPosition()   — 实际位置（0x6064，pp）
 *   getVelocity()   — 实际速度（0x606C，pp/s）
 *   getTorque()     — 实际力矩（0x6077，‰ 额定力矩）
 *   getStatusWord() — 状态字（0x6041）
 *
 * 典型使用：
 *   CanopenMotorDriver drv("can0", 1);
 *   drv.init();
 *   drv.enable();
 *   drv.setProfilePositionMode(vel, accel, decel);
 *   drv.moveToPosition(target_pp, false, false);
 *
 * 权限要求：
 *   进程需要 CAP_NET_RAW，编译后执行一次：
 *   sudo setcap cap_net_raw+ep <可执行文件路径>
 *
 * 所属模块：hardware_driver/include/canopen_motor_driver/
 *
 * @version 1.1
 * @date 2026-05-27
 * @copyright Copyright (c) 2026 EMEET
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace arm {

class CanopenMotorDriver {
public:
    /**
     * @brief 构造函数
     * @param ifname        SocketCAN 接口名，如 "can0"
     * @param node_id       CANopen 从站节点 ID（1~127）
     * @param sdo_timeout_ms SDO 请求超时时间（毫秒），默认 500ms
     */
    explicit CanopenMotorDriver(const std::string& ifname, uint8_t node_id,
                                int sdo_timeout_ms = 500);
    ~CanopenMotorDriver();

    /** @brief 打开并绑定 SocketCAN 接口，失败返回 false */
    bool init();

    /** @brief 关闭 SocketCAN 接口 */
    void close();

    /** @brief 接口是否已打开 */
    bool isOpen() const { return fd_ >= 0; }

    /** @brief 返回节点 ID */
    uint8_t nodeId() const { return node_id_; }

    // ── NMT 网络管理 ─────────────────────────────────────────────────────────
    /**
     * @brief 发送 NMT 命令
     * @param cmd 命令字：0x01=Start  0x02=Stop  0x80=PreOp
     *                    0x81=ResetNode  0x82=ResetComm
     */
    bool nmtCommand(uint8_t cmd);

    /**
     * @brief 将当前参数保存到 EEPROM（0x1010:02 = "save"）
     * @param timeout_ms SDO 超时，默认 3000ms（EEPROM 写入比普通 SDO 慢）
     * @note  部分驱动器要求先切到 NMT 预操作状态，调用前请自行 nmtCommand(0x80)
     */
    bool saveParameters(int timeout_ms = 3000);

    // ── SDO 对象字典读写（expedited 阻塞模式）────────────────────────────────
    /**
     * @brief SDO 写（expedited download，阻塞等待响应）
     * @param index    对象字典索引，如 0x6040
     * @param subindex 子索引
     * @param data     待写数据（小端序）
     * @param len      数据长度（1~4 字节）
     * @return true=从站返回成功；false=超时或从站返回 Abort
     */
    bool writeSDO(uint16_t index, uint8_t subindex,
                  const void* data, uint8_t len);

    /**
     * @brief SDO 读（expedited upload，阻塞等待响应）
     * @param index    对象字典索引
     * @param subindex 子索引
     * @param data     读出数据缓冲区（至少 4 字节）
     * @param len      [out] 实际读出字节数
     * @return true=读取成功；false=超时或从站返回 Abort
     */
    bool readSDO(uint16_t index, uint8_t subindex,
                 void* data, uint8_t& len);

    // ── DS402 伺服状态机 ─────────────────────────────────────────────────────
    /**
     * @brief 故障复位（对 6040h bit7 产生上升沿）
     * @note  复位后需重新调用 enable()
     */
    bool resetFault();

    /**
     * @brief 使能伺服（DS402 三步状态机：Shutdown→SwitchOn→EnableOperation）
     * @param timeout_ms 总超时（三步各分得 1/3），默认 3000ms
     */
    bool enable(int timeout_ms = 3000);

    /**
     * @brief 禁用伺服（回到 Shutdown 状态，电机失力矩）
     */
    bool disable();

    // ── 轮廓位置模式 1-PP ────────────────────────────────────────────────────
    /**
     * @brief 切换到轮廓位置模式（PP）并配置运动参数
     * @param vel   轮廓速度（指令单位/s），默认 1000
     * @param accel 加速度（指令单位/s²），默认 500
     * @param decel 减速度（指令单位/s²），默认 500
     * @note  需在 enable() 之后调用
     */
    bool setProfilePositionMode(uint32_t vel   = 1000,
                                uint32_t accel = 500,
                                uint32_t decel = 500);

    /**
     * @brief 单独更新 PP 模式的轮廓速度 0x6081（不重设模式/加减速）
     * @param vel 轮廓速度（指令单位/s）
     * @note  在 setProfilePositionMode() 之后逐 waypoint 调用，让 PP 跟随规划速度
     */
    bool setProfileVelocity(uint32_t vel);

    /**
     * @brief 发送位置指令
     * @param pos        目标位置（指令单位 pp）
     * @param relative   true=相对当前位置，false=绝对位置
     * @param wait_done  true=阻塞直到到位，false=触发后立即返回
     * @param timeout_ms 等待到位超时（毫秒）
     */
    bool moveToPosition(int32_t pos,
                        bool    relative   = false,
                        bool    wait_done  = true,
                        int     timeout_ms = 10000);

    // ── 轮廓速度模式 3-PV ────────────────────────────────────────────────────
    /**
     * @brief 切换到轮廓速度模式（PV）并配置加减速
     * @param accel 加速度（指令单位/s²），默认 500
     * @param decel 减速度（指令单位/s²），默认 500
     */
    bool setProfileVelocityMode(uint32_t accel = 500,
                                uint32_t decel = 500);

    /**
     * @brief 设置目标速度（持续运转）
     * @param vel_pps 目标速度（指令单位/s），正值正转，负值反转，0 停止
     */
    bool setTargetVelocity(int32_t vel_pps);

    // ── 轮廓力矩模式 4-PT ────────────────────────────────────────────────────
    /**
     * @brief 切换到轮廓力矩模式（PT）并配置限制参数
     * @param max_torque_permille 最大力矩（0x6072），单位 0.1% 额定力矩，范围 0~3000
     * @param torque_slope_ms     力矩斜坡时间（0x6087），单位 ms，0=立即到达
     * @param max_vel             最大速度限制（0x607F），单位 pp/s，必须 > 0 否则电机不动
     */
    bool setProfileTorqueMode(uint16_t max_torque_permille = 1000,
                              uint32_t torque_slope_ms     = 0,
                              uint32_t max_vel             = 50000);

    /**
     * @brief 设置目标力矩
     * @param torque_permille 目标力矩（0x6071），单位 0.1% 额定力矩，范围 -3000~3000
     *                        正值正转，负值反转，0 停止
     */
    bool setTargetTorque(int16_t torque_permille);

    // ── 插补位置模式 7-IP ────────────────────────────────────────────────────
    /**
     * @brief 配置并切换到插补位置模式（IP）
     * @param period_ms 插补周期（毫秒），主站须按此周期写入位置点，默认 10ms
     * @param max_vel   最大速度限制（0x607F，pp/s），防止速度限制为 0 导致不动
     * @note  内部会读取当前位置写入 0x60C1 作为初始点，再使能 bit4，避免跳变
     */
    bool setInterpolatedPositionMode(uint8_t period_ms = 10,
                                     uint32_t max_vel  = 50000);

    /**
     * @brief 写入一个插补位置点（须以配置周期定期调用）
     * @param pos_pp 目标位置（指令单位 pp），绝对位置
     */
    bool setInterpolatedPosition(int32_t pos_pp);

    /** @brief 发送 RPDO1 数据帧（含控制字 + 位置，不含 SYNC），用于多轴同步 */
    bool sendInterpolationData(int32_t pos_pp);
    /** @brief 发送 SYNC 广播帧，触发所有同步 PDO 同时执行 */
    bool sendSYNC();

    // ── 回零模式 6-HM ────────────────────────────────────────────────────────
    /**
     * @brief 配置并切换到回零模式（HM）
     * @param method   回零方式（0x6098），1~35，见手册 3.4 节
     * @param fast_vel 高速搜索速度（pp/s），对应 0x6099:01
     * @param slow_vel 低速搜索速度（pp/s），对应 0x6099:02，须 ≥ 1
     * @param accel    回零加减速（pp/s²），对应 0x609A
     * @param offset   原点偏置（pp），对应 0x607C，默认 0
     */
    bool setHomingMode(uint8_t  method,
                       uint32_t fast_vel,
                       uint32_t slow_vel,
                       uint32_t accel,
                       int32_t  offset = 0);

    /**
     * @brief 触发回零动作，阻塞等待完成
     * @param timeout_ms    最大等待时间（毫秒）
     * @param on_heartbeat  等待期间每 ~80ms 调用一次的回调（用于维持心跳），可为 nullptr
     * @return true=回零完成；false=超时或回零错误（状态字 bit13）
     * @note 须在 enable() 和 setHomingMode() 之后调用
     */
    bool startHoming(int timeout_ms = 30000,
                     std::function<void()> on_heartbeat = nullptr);

    // ── 心跳生产者 ───────────────────────────────────────────────────────────
    /**
     * @brief 发送主站心跳帧（0x700 + master_node_id，data=0x05）
     * @param master_node_id 主站节点 ID，通常为 127（0x7F）
     * @note  需周期性调用（≤ 驱动器 0x1016 配置的超时时间的一半），否则驱动器
     *        触发 Heartbeat Event 故障（紧急报文 0x8130）
     */
    bool sendHeartbeat(uint8_t master_node_id);

    // ── 反馈读取 ─────────────────────────────────────────────────────────────
    /** @brief 读取当前位置（6064h，指令单位 pp） */
    int32_t getPosition();

    /** @brief 读取当前速度（606Ch，指令单位 pp/s） */
    int32_t getVelocity();

    /** @brief 读取实际力矩（6077h），单位：额定力矩的千分之一（‰），正负表示方向 */
    int16_t getTorque();

    /** @brief 读取状态字（6041h），bit3=故障，bit10=目标到达 */
    uint16_t getStatusWord();

private:
    // SDO 类型化写辅助
    bool writeSDOu8 (uint16_t idx, uint8_t sub, uint8_t  v);
    bool writeSDOu16(uint16_t idx, uint8_t sub, uint16_t v);
    bool writeSDOu32(uint16_t idx, uint8_t sub, uint32_t v);
    bool writeSDOi32(uint16_t idx, uint8_t sub, int32_t  v);

    // SDO 类型化读辅助
    bool readSDOu16(uint16_t idx, uint8_t sub, uint16_t& out);
    bool readSDOi32(uint16_t idx, uint8_t sub, int32_t&  out);

    /** @brief 轮询状态字直到 (sw & mask) == expected 或超时 */
    bool waitStatusWord(uint16_t mask, uint16_t expected, int timeout_ms);

    // IP 模式 PDO 辅助
    /** @brief 配置 RPDO1 映射为 0x6040 + 0x60C1:01（控制字 + 插补位置）*/
    bool configureIPModePDO();
    /** @brief 通过 RPDO1 PDO 帧发送插补位置 + SYNC，单轴场景快捷方法 */
    bool sendInterpolationPDO(int32_t pos_pp);

    std::string ifname_;         ///< SocketCAN 接口名
    uint8_t     node_id_;        ///< CANopen 节点 ID
    int         sdo_timeout_ms_; ///< SDO 超时（毫秒）
    int         fd_{-1};         ///< SocketCAN 文件描述符，-1 表示未打开
};

} // namespace arm
