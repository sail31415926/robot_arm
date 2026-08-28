/**
 * @file trajectory_store.cpp
 * @brief TrajectoryStore 实现 —— yaml-cpp 序列化 + 原子写入 + 目录枚举
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#include "robot_arm_teach/trajectory_store.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <system_error>

#include <yaml-cpp/yaml.h>

#include "robot_arm_teach/msg/motion_meta.hpp"
#include "robot_arm_teach/msg/motion_type.hpp"

namespace robot_arm_teach
{

namespace fs = std::filesystem;

namespace
{
using MotionMetaMsg = robot_arm_teach::msg::MotionMeta;

constexpr const char * FILE_EXT = ".yaml";

// ── 枚举 ⇄ 字符串（理由见头文件约定①）──────────────────────────────────────
std::string follow_policy_to_string(uint8_t v)
{
  switch (v) {
    case MotionMetaMsg::FOLLOW_LOCK_SUBJECT: return "LOCK_SUBJECT";
    case MotionMetaMsg::FOLLOW_PARALLEL:     return "PARALLEL";
    default:                                 return "NONE";
  }
}
uint8_t follow_policy_from_string(const std::string & s)
{
  if (s == "LOCK_SUBJECT") return MotionMetaMsg::FOLLOW_LOCK_SUBJECT;
  if (s == "PARALLEL")     return MotionMetaMsg::FOLLOW_PARALLEL;
  return MotionMetaMsg::FOLLOW_NONE;
}

std::string arc_rotation_to_string(uint8_t v)
{
  return v == MotionMetaMsg::ARC_CCW ? "CCW" : "CW";
}
uint8_t arc_rotation_from_string(const std::string & s)
{
  return s == "CCW" ? MotionMetaMsg::ARC_CCW : MotionMetaMsg::ARC_CW;
}

std::string crane_dir_to_string(uint8_t v)
{
  return v == MotionMetaMsg::CRANE_DOWN ? "DOWN" : "UP";
}
uint8_t crane_dir_from_string(const std::string & s)
{
  return s == "DOWN" ? MotionMetaMsg::CRANE_DOWN : MotionMetaMsg::CRANE_UP;
}

// ── 定长数组 ⇄ YAML 序列 ────────────────────────────────────────────────────
template <size_t N>
YAML::Node array_to_node(const std::array<double, N> & a)
{
  YAML::Node n(YAML::NodeType::Sequence);
  n.SetStyle(YAML::EmitterStyle::Flow);   // 一行一个数组，文件可读性好得多
  for (double v : a) n.push_back(v);
  return n;
}

// 长度不符时按较短者填、其余保持 0 —— 但调用方（load）会把长度不符当错误上报，
// 这里只保证不越界写。
template <size_t N>
bool node_to_array(const YAML::Node & n, std::array<double, N> * out)
{
  if (!out) return false;
  out->fill(0.0);
  if (!n || !n.IsSequence()) return false;
  if (n.size() != N) return false;
  for (size_t i = 0; i < N; ++i) (*out)[i] = n[i].as<double>();
  return true;
}
}  // namespace

TrajectoryStore::TrajectoryStore(std::string directory)
: directory_(std::move(directory))
{
}

TrajectoryStore::Result TrajectoryStore::ensure_directory() const
{
  std::error_code ec;
  if (fs::exists(directory_, ec)) {
    if (fs::is_directory(directory_, ec)) return Result{true, "", directory_};
    return Result{false, "存储路径已存在但不是目录：" + directory_, directory_};
  }
  fs::create_directories(directory_, ec);
  if (ec) {
    return Result{false, "创建存储目录失败：" + directory_ + "（" + ec.message() + "）",
                  directory_};
  }
  return Result{true, "", directory_};
}

std::string TrajectoryStore::path_for(const std::string & name) const
{
  if (!is_valid_trajectory_name(name)) return "";
  return (fs::path(directory_) / (name + FILE_EXT)).string();
}

bool TrajectoryStore::exists(const std::string & name) const
{
  const std::string p = path_for(name);
  if (p.empty()) return false;
  std::error_code ec;
  return fs::exists(p, ec) && fs::is_regular_file(p, ec);
}

TrajectoryStore::Result TrajectoryStore::save(const TeachTrajectoryMsg & traj,
                                              bool overwrite) const
{
  const std::string name = traj.name;
  if (!is_valid_trajectory_name(name)) {
    return Result{false,
                  "轨迹名不合法：只允许 [A-Za-z0-9_-]、长度 1..64（收到「" + name + "」）", ""};
  }
  if (auto r = ensure_directory(); !r) return r;

  const std::string path = path_for(name);
  std::error_code ec;
  if (!overwrite && fs::exists(path, ec)) {
    return Result{false, "同名轨迹已存在，未覆盖（overwrite=true 才覆盖）：" + path, path};
  }

  YAML::Node root;
  root["format_version"] = traj.format_version;
  root["name"]           = traj.name;
  root["created_at"]     = traj.created_at;
  root["description"]    = traj.description;

  YAML::Node names(YAML::NodeType::Sequence);
  names.SetStyle(YAML::EmitterStyle::Flow);
  for (const auto & n : traj.joint_names) names.push_back(n);
  root["joint_names"] = names;

  root["motion_type"]        = motion_type_to_string(traj.motion_type.value);
  root["teach_mode"]         = teach_mode_to_string(traj.teach_mode);
  root["teach_control_mode"] = static_cast<int>(traj.teach_control_mode.mode);
  root["sample_rate_hz"]     = traj.sample_rate_hz;
  root["raw_point_count"]    = traj.raw_point_count;
  root["compress_position_eps"] = traj.compress_position_eps;
  root["duration_sec"]       = traj.duration_sec;

  root["start_positions"]  = array_to_node(traj.start_positions);
  root["end_positions"]    = array_to_node(traj.end_positions);
  root["max_velocity"]     = array_to_node(traj.max_velocity);
  root["max_acceleration"] = array_to_node(traj.max_acceleration);

  const auto & m = traj.motion_meta;
  YAML::Node meta;
  meta["subject_label"] = m.subject_label;
  meta["dolly"]["direction"]          = array_to_node(m.dolly_direction);
  meta["dolly"]["distance_m"]         = m.dolly_distance_m;
  meta["dolly"]["keep_camera_facing"] = m.dolly_keep_camera_facing;
  meta["truck"]["direction"]     = array_to_node(m.truck_direction);
  meta["truck"]["distance_m"]    = m.truck_distance_m;
  meta["truck"]["follow_policy"] = follow_policy_to_string(m.truck_follow_policy);
  meta["arc"]["target_point"]    = array_to_node(m.arc_target_point);
  meta["arc"]["radius_m"]        = m.arc_radius_m;
  meta["arc"]["start_angle_deg"] = m.arc_start_angle_deg;
  meta["arc"]["end_angle_deg"]   = m.arc_end_angle_deg;
  meta["arc"]["rotation"]        = arc_rotation_to_string(m.arc_rotation_direction);
  meta["crane"]["height_delta_m"] = m.crane_height_delta_m;
  meta["crane"]["direction"]      = crane_dir_to_string(m.crane_direction);
  root["motion_meta"] = meta;

  YAML::Node points(YAML::NodeType::Sequence);
  for (const auto & p : traj.points) {
    YAML::Node n;
    n.SetStyle(YAML::EmitterStyle::Flow);   // 一行一个采样点，几千点的文件仍能用眼睛扫
    n["t"] = p.time_from_start;
    n["p"] = array_to_node(p.positions);
    n["v"] = array_to_node(p.velocities);
    points.push_back(n);
  }
  root["points"] = points;

  // 原子写入：临时文件 + rename（见头文件约定②）
  const std::string tmp = path + ".tmp";
  {
    std::ofstream ofs(tmp, std::ios::binary | std::ios::trunc);
    if (!ofs) return Result{false, "无法写入临时文件：" + tmp, path};
    ofs << "# robot_arm_teach 示教轨迹  格式说明见 robot_arm_teach/doc/轨迹格式说明.md\n";
    ofs << YAML::Dump(root) << "\n";
    if (!ofs) { ofs.close(); fs::remove(tmp, ec); return Result{false, "写入失败：" + tmp, path}; }
  }
  fs::rename(tmp, path, ec);
  if (ec) {
    fs::remove(tmp, ec);
    return Result{false, "原子替换失败：" + path + "（" + ec.message() + "）", path};
  }
  return Result{true, "", path};
}

TrajectoryStore::Result TrajectoryStore::load(const std::string & name,
                                              TeachTrajectoryMsg * out) const
{
  if (!out) return Result{false, "内部错误：out 为空", ""};
  if (!is_valid_trajectory_name(name)) {
    return Result{false, "轨迹名不合法：只允许 [A-Za-z0-9_-]、长度 1..64（收到「" + name + "」）",
                  ""};
  }
  const std::string path = path_for(name);
  std::error_code ec;
  if (!fs::exists(path, ec)) return Result{false, "轨迹不存在：" + path, path};

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    return Result{false, std::string("YAML 解析失败：") + e.what(), path};
  }

  try {
    TeachTrajectoryMsg traj;
    traj.format_version = root["format_version"].as<uint16_t>(0);
    if (traj.format_version == 0 || traj.format_version > TeachTrajectoryMsg::FORMAT_VERSION) {
      return Result{false,
                    "格式版本 " + std::to_string(traj.format_version) + " 不受支持（本节点支持 1.." +
                        std::to_string(TeachTrajectoryMsg::FORMAT_VERSION) + "）",
                    path};
    }
    traj.name        = root["name"].as<std::string>(name);
    traj.created_at  = root["created_at"].as<std::string>("");
    traj.description = root["description"].as<std::string>("");

    if (!root["joint_names"] || !root["joint_names"].IsSequence()) {
      return Result{false, "缺少 joint_names", path};
    }
    for (const auto & n : root["joint_names"]) traj.joint_names.push_back(n.as<std::string>());

    uint8_t mt = 0;
    const std::string mt_text = root["motion_type"].as<std::string>("FREEFORM");
    if (!motion_type_from_string(mt_text, &mt)) {
      return Result{false, "未知的 motion_type：" + mt_text, path};
    }
    traj.motion_type.value = mt;

    uint8_t tm = 0;
    const std::string tm_text = root["teach_mode"].as<std::string>("JOG");
    if (!teach_mode_from_string(tm_text, &tm)) {
      return Result{false, "未知的 teach_mode：" + tm_text, path};
    }
    traj.teach_mode = tm;

    traj.teach_control_mode.mode =
        static_cast<uint8_t>(root["teach_control_mode"].as<int>(0));
    traj.sample_rate_hz        = root["sample_rate_hz"].as<double>(0.0);
    traj.raw_point_count       = root["raw_point_count"].as<uint32_t>(0);
    traj.compress_position_eps = root["compress_position_eps"].as<double>(0.0);
    traj.duration_sec          = root["duration_sec"].as<double>(0.0);

    node_to_array(root["start_positions"], &traj.start_positions);
    node_to_array(root["end_positions"], &traj.end_positions);
    node_to_array(root["max_velocity"], &traj.max_velocity);
    node_to_array(root["max_acceleration"], &traj.max_acceleration);

    if (const YAML::Node meta = root["motion_meta"]) {
      auto & m = traj.motion_meta;
      m.subject_label = meta["subject_label"].as<std::string>("");
      if (const YAML::Node d = meta["dolly"]) {
        node_to_array(d["direction"], &m.dolly_direction);
        m.dolly_distance_m         = d["distance_m"].as<double>(0.0);
        m.dolly_keep_camera_facing = d["keep_camera_facing"].as<bool>(false);
      }
      if (const YAML::Node t = meta["truck"]) {
        node_to_array(t["direction"], &m.truck_direction);
        m.truck_distance_m    = t["distance_m"].as<double>(0.0);
        m.truck_follow_policy = follow_policy_from_string(
            t["follow_policy"].as<std::string>("NONE"));
      }
      if (const YAML::Node a = meta["arc"]) {
        node_to_array(a["target_point"], &m.arc_target_point);
        m.arc_radius_m        = a["radius_m"].as<double>(0.0);
        m.arc_start_angle_deg = a["start_angle_deg"].as<double>(0.0);
        m.arc_end_angle_deg   = a["end_angle_deg"].as<double>(0.0);
        m.arc_rotation_direction = arc_rotation_from_string(a["rotation"].as<std::string>("CW"));
      }
      if (const YAML::Node c = meta["crane"]) {
        m.crane_height_delta_m = c["height_delta_m"].as<double>(0.0);
        m.crane_direction      = crane_dir_from_string(c["direction"].as<std::string>("UP"));
      }
    }

    if (!root["points"] || !root["points"].IsSequence()) {
      return Result{false, "缺少 points", path};
    }
    traj.points.reserve(root["points"].size());
    size_t idx = 0;
    for (const auto & n : root["points"]) {
      robot_arm_teach::msg::TeachPoint p;
      p.time_from_start = n["t"].as<double>(std::numeric_limits<double>::quiet_NaN());
      if (!node_to_array(n["p"], &p.positions) || !node_to_array(n["v"], &p.velocities)) {
        return Result{false,
                      "第 " + std::to_string(idx) + " 个点的 p / v 必须各 " +
                          std::to_string(TEACH_JOINT_COUNT) + " 个数",
                      path};
      }
      traj.points.push_back(p);
      ++idx;
    }

    *out = std::move(traj);
    return Result{true, "", path};
  } catch (const std::exception & e) {
    return Result{false, std::string("YAML 字段读取失败：") + e.what(), path};
  }
}

TrajectoryStore::Result TrajectoryStore::remove(const std::string & name) const
{
  if (!is_valid_trajectory_name(name)) {
    return Result{false, "轨迹名不合法：只允许 [A-Za-z0-9_-]、长度 1..64（收到「" + name + "」）",
                  ""};
  }
  const std::string path = path_for(name);
  std::error_code ec;
  if (!fs::exists(path, ec)) return Result{false, "轨迹不存在：" + path, ""};
  if (!fs::remove(path, ec) || ec) {
    return Result{false, "删除失败：" + path + "（" + ec.message() + "）", ""};
  }
  return Result{true, "", path};
}

TrajectoryStore::Result TrajectoryStore::list(std::vector<Summary> * out,
                                              std::vector<std::string> * invalid) const
{
  if (out) out->clear();
  if (invalid) invalid->clear();

  std::error_code ec;
  if (!fs::exists(directory_, ec)) {
    // 目录还没建（一条都没存过）不是错误，返回空列表
    return Result{true, "存储目录尚不存在（还没保存过轨迹）：" + directory_, directory_};
  }

  std::vector<std::string> names;
  for (const auto & entry : fs::directory_iterator(directory_, ec)) {
    if (ec) break;
    if (!entry.is_regular_file()) continue;
    if (entry.path().extension() != FILE_EXT) continue;
    names.push_back(entry.path().stem().string());
  }
  std::sort(names.begin(), names.end());

  for (const auto & n : names) {
    // 摘要只需头部字段，但 yaml-cpp 没有流式部分解析，只能整份 Load ——
    // 所以这里仍然会读完文件，只是**不把 points 拷进消息**（那才是内存与应答的大头）。
    YAML::Node root;
    try {
      root = YAML::LoadFile(path_for(n));
    } catch (const std::exception &) {
      if (invalid) invalid->push_back(n);
      continue;
    }
    uint8_t mt = 0;
    const uint16_t ver = root["format_version"].as<uint16_t>(0);
    if (ver == 0 || ver > TeachTrajectoryMsg::FORMAT_VERSION ||
        !motion_type_from_string(root["motion_type"].as<std::string>("?"), &mt))
    {
      if (invalid) invalid->push_back(n);
      continue;
    }
    Summary s;
    s.name         = root["name"].as<std::string>(n);
    s.created_at   = root["created_at"].as<std::string>("");
    s.motion_type  = mt;
    s.point_count  = root["points"] && root["points"].IsSequence()
                         ? static_cast<uint32_t>(root["points"].size()) : 0u;
    s.duration_sec = root["duration_sec"].as<double>(0.0);
    if (out) out->push_back(std::move(s));
  }
  return Result{true, "", directory_};
}

}  // namespace robot_arm_teach
