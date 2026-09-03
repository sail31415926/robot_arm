/**
 * @file geometry.hpp
 * @brief 四元数 / 球坐标 / 相机朝向数学（C++, header-only）
 *
 * 1:1 移植自 tools/arm_utils.py：rpy_to_quat / quat_to_rpy / aim_quat /
 * sphere_to_cart / cart_to_sphere / theta_ref。四元数分量序 (x,y,z,w)，与 Python
 * 完全一致（约定改变会导致运动方向变化）。纯数学、无 ROS 依赖，供 motion / commander 复用。
 *
 * @version 1.0
 * @date 2026-07-01
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace robot_arm_node::motion
{

using Quat = std::array<double, 4>;   // (qx, qy, qz, qw)
using Vec3 = std::array<double, 3>;   // (x, y, z)
using Rpy  = std::array<double, 3>;   // (roll, pitch, yaw)  rad

// RPY(rad) → (qx,qy,qz,qw)，ZYX 内旋 / XYZ 外旋
/**
 * @brief RPY 欧拉角转四元数（ZYX 内旋 / XYZ 外旋）。
 *
 * 分量序为 (x, y, z, w)，与 Python 侧 arm_utils 完全一致 —— 改约定会导致
 * 运动方向变化，勿动。
 *
 * @param roll 绕 X 轴转角（rad）。
 * @param pitch 绕 Y 轴转角（rad）。
 * @param yaw 绕 Z 轴转角（rad）。
 * @return 四元数 (qx, qy, qz, qw)。
 */
inline Quat rpy_to_quat(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5),  sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5),   sy = std::sin(yaw * 0.5);
  return {sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy,
          cr * cp * cy + sr * sp * sy};
}

// (qx,qy,qz,qw) → (roll,pitch,yaw) rad
/**
 * @brief 四元数转 RPY 欧拉角。
 *
 * pitch 的 asin 参数做了 clamp，避免数值误差让 |sin|>1 时返回 NaN。
 *
 * @param x 四元数 x 分量。
 * @param y 四元数 y 分量。
 * @param z 四元数 z 分量。
 * @param w 四元数 w 分量。
 * @return (roll, pitch, yaw)，单位 rad。
 */
inline Rpy quat_to_rpy(double x, double y, double z, double w)
{
  const double roll  = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const double sp    = std::clamp(2 * (w * y - z * x), -1.0, 1.0);
  const double pitch = std::asin(sp);
  const double yaw   = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {roll, pitch, yaw};
}

// 主体→世界原点在 XY 平面的方位角，作 θ=0 参考方向（近侧）
/**
 * @brief 球坐标 θ=0 的参考方向：主体指向世界原点在 XY 平面的方位角。
 *
 * 取「近侧」为 θ=0，使运镜的角度参数与直觉一致（θ 增大 = 绕着转开）。
 *
 * @param ox 球心 x 坐标。
 * @param oy 球心 y 坐标。
 * @return 参考方位角（rad）。
 */
inline double theta_ref(double ox, double oy) { return std::atan2(-oy, -ox); }

// Z-up 球坐标 → 世界系笛卡尔位置
/**
 * @brief Z-up 球坐标转世界系笛卡尔位置。
 *
 * θ 以 theta_ref 给出的近侧方向为 0，φ 为仰角（非极角），r 为半径。
 *
 * @param theta_rad 方位角（rad，相对近侧参考方向）。
 * @param phi_rad 仰角（rad）。
 * @param r 半径（m）。
 * @param ox 球心 x 坐标（m）。
 * @param oy 球心 y 坐标（m）。
 * @param oz 球心 z 坐标（m）。
 * @return 世界系位置 (x, y, z)。
 */
inline Vec3 sphere_to_cart(double theta_rad, double phi_rad, double r,
                           double ox, double oy, double oz)
{
  const double tw = theta_ref(ox, oy) + theta_rad;
  const double cp = std::cos(phi_rad);
  return {ox + r * cp * std::cos(tw),
          oy + r * cp * std::sin(tw),
          oz + r * std::sin(phi_rad)};
}

// 笛卡尔 → Z-up 球坐标（θ 以近侧为 0）
/**
 * @brief 世界系笛卡尔位置转 Z-up 球坐标（sphere_to_cart 的逆）。
 *
 * θ 归一化到 (-π, π]；点与球心重合（r < 1e-9）时返回全零而非 NaN。
 *
 * @param px 点 x 坐标（m）。
 * @param py 点 y 坐标（m）。
 * @param pz 点 z 坐标（m）。
 * @param ox 球心 x 坐标（m）。
 * @param oy 球心 y 坐标（m）。
 * @param oz 球心 z 坐标（m）。
 * @return (theta, phi, r)，角度单位 rad、半径单位 m。
 */
inline Vec3 cart_to_sphere(double px, double py, double pz,
                           double ox, double oy, double oz)
{
  const double dx = px - ox, dy = py - oy, dz = pz - oz;
  const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (r < 1e-9) return {0.0, 0.0, 0.0};
  const double phi = std::asin(std::clamp(dz / r, -1.0, 1.0));
  double theta = std::atan2(dy, dx) - theta_ref(ox, oy);
  theta = std::fmod(theta + M_PI, 2 * M_PI);
  if (theta < 0) theta += 2 * M_PI;
  theta -= M_PI;
  return {theta, phi, r};   // (theta, phi, r)
}

// 画面水平所需的 EEF roll（rad）——必须与 Python 侧 arm_utils.EEF_LEVEL_ROLL 保持一致。
//
// 推导：设 R_c = R(EEF→相机光学系)。画面水平 ⇔ 图像右向量（光学 +X）水平
//       ⇔ (R_eef · R_c)[2][0] == 0，其中 R_eef = Rz(yaw)Ry(pitch)Rx(roll)。
//       对任意 pitch/yaw 都成立的 roll 即本常量。
//
//   V1（tool0 → camera_optical_frame，rpy = 0, -π/2, π）  → roll = π/2  ✔ 旧值
//   V2（gimbal_tool0 → Cam0，       rpy = -π/2, 0, -π/2） → roll = 0
//
// 2026-07-29：云台换 V2 后仍用 π/2，导致环绕运镜画面歪斜 90°，且逼云台 roll 轴
// （Joint5，±1.5 rad）去凑该姿态 → IK 大面积无解，故改为 0。
// 两代的光轴都是 EEF 的 +X 轴，所以下面 pitch/yaw 的算法不变。
inline constexpr double EEF_LEVEL_ROLL = 0.0;

// EEF X 轴朝向目标（= 相机光轴指向目标），roll 取 EEF_LEVEL_ROLL 保证画面水平
/**
 * @brief 求让相机光轴（EEF 的 +X 轴）指向目标点的姿态四元数。
 *
 * roll 固定取 EEF_LEVEL_ROLL 以保证画面水平（该常量的推导见其上方注释）。
 * 相机与目标重合（距离 < 1e-9）时退化为只给水平 roll 的零姿态。
 *
 * @param cam_x 相机位置 x（m）。
 * @param cam_y 相机位置 y（m）。
 * @param cam_z 相机位置 z（m）。
 * @param tgt_x 目标位置 x（m）。
 * @param tgt_y 目标位置 y（m）。
 * @param tgt_z 目标位置 z（m）。
 * @return 朝向目标的姿态四元数 (qx, qy, qz, qw)。
 */
inline Quat aim_quat(double cam_x, double cam_y, double cam_z,
                     double tgt_x, double tgt_y, double tgt_z)
{
  double dx = tgt_x - cam_x, dy = tgt_y - cam_y, dz = tgt_z - cam_z;
  const double n = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (n < 1e-9) return rpy_to_quat(EEF_LEVEL_ROLL, 0.0, 0.0);
  dx /= n; dy /= n; dz /= n;
  const double pitch = std::asin(std::clamp(-dz, -1.0, 1.0));
  const double yaw   = std::atan2(dy, dx);
  return rpy_to_quat(EEF_LEVEL_ROLL, pitch, yaw);
}

}  // namespace robot_arm_node::motion
