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
inline Rpy quat_to_rpy(double x, double y, double z, double w)
{
  const double roll  = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const double sp    = std::clamp(2 * (w * y - z * x), -1.0, 1.0);
  const double pitch = std::asin(sp);
  const double yaw   = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {roll, pitch, yaw};
}

// 主体→世界原点在 XY 平面的方位角，作 θ=0 参考方向（近侧）
inline double theta_ref(double ox, double oy) { return std::atan2(-oy, -ox); }

// Z-up 球坐标 → 世界系笛卡尔位置
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

// EEF X 轴朝向目标、roll 固定 90°（相机光轴沿 EEF X 的安装方式）
inline Quat aim_quat(double cam_x, double cam_y, double cam_z,
                     double tgt_x, double tgt_y, double tgt_z)
{
  double dx = tgt_x - cam_x, dy = tgt_y - cam_y, dz = tgt_z - cam_z;
  const double n = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (n < 1e-9) return rpy_to_quat(M_PI / 2, 0.0, 0.0);
  dx /= n; dy /= n; dz /= n;
  const double pitch = std::asin(std::clamp(-dz, -1.0, 1.0));
  const double yaw   = std::atan2(dy, dx);
  return rpy_to_quat(M_PI / 2, pitch, yaw);
}

}  // namespace robot_arm_node::motion
