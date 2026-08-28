/**
 * @file trajectory_store.hpp
 * @brief 轨迹持久化 —— YAML 文件的读 / 写 / 列 / 删（纯逻辑，可单元测试）
 *
 * 一条轨迹 = 一个 YAML 文件 `<directory>/<name>.yaml`。格式见 doc/轨迹格式说明.md。
 *
 * 三条设计约定：
 *   ① **枚举存名字不存数字**（motion_type: DOLLY，不是 motion_type: 1）。
 *      存数字的话哪天枚举值变了，老文件会被**静默**按新语义读出来 —— 一条 DOLLY 变成
 *      TRUCK 不会报任何错，只是元数据全错。存名字读不出来会明确失败。
 *   ② **原子写入**：先写 `<name>.yaml.tmp` 再 rename。50Hz 录几分钟的文件不小，
 *      写一半断电/被 Ctrl-C 会留下半截 YAML，下次加载解析失败还得人去删。
 *   ③ **名字白名单后才拼路径**（is_valid_trajectory_name）。名字来自服务请求，
 *      不挡 "/" 和 ".." 的话 save/delete 能操作到目录外的任意文件。
 *
 * list() 刻意只读头部字段、不解析 points：目录里堆着几十条长轨迹时，
 * 一次全量加载会把进程内存和服务应答一起撑爆。
 *
 * @version 1.0
 * @date 2026-08-24
 * @copyright Copyright (c) 2026 eMeet
 */
#pragma once

#include <string>
#include <vector>

#include "robot_arm_teach/msg/teach_trajectory.hpp"
#include "robot_arm_teach/teach_types.hpp"

namespace robot_arm_teach
{

class TrajectoryStore
{
public:
  struct Result
  {
    bool        ok{false};
    std::string message;
    std::string path;

    explicit operator bool() const { return ok; }
  };

  // 轨迹摘要（list 用，不含 points）
  struct Summary
  {
    std::string name;
    std::string created_at;
    uint8_t     motion_type{0};
    uint32_t    point_count{0};
    double      duration_sec{0.0};
  };

  explicit TrajectoryStore(std::string directory);

  const std::string & directory() const { return directory_; }
  void set_directory(std::string directory) { directory_ = std::move(directory); }

  // 目录不存在则创建（含父级）。启动时调一次；每次 save 也会兜底调用。
  Result ensure_directory() const;

  // 名字白名单校验后拼出的绝对/相对路径（不保证文件存在）
  std::string path_for(const std::string & name) const;

  bool exists(const std::string & name) const;

  // 写入。traj.name 为空时用 name 参数；两者都为空则失败。
  // overwrite=false 且文件已存在 → 失败（不覆盖）。
  Result save(const TeachTrajectoryMsg & traj, bool overwrite) const;

  // 读取。解析失败 / 版本不支持 → ok=false，*out 不保证内容。
  Result load(const std::string & name, TeachTrajectoryMsg * out) const;

  Result remove(const std::string & name) const;

  // 列目录。解析失败的文件名（不含扩展名）写进 invalid，便于人工排查坏文件。
  Result list(std::vector<Summary> * out, std::vector<std::string> * invalid) const;

private:
  std::string directory_;
};

}  // namespace robot_arm_teach
