# Vendored: ros2_canopen（精简版）

- 上游: https://github.com/ros-industrial/ros2_canopen.git
- 分支: humble
- 提交: fef50e54b1c94c50e908e2c5d0b8888eed907e8d (0.2.13-5-gfef50e54, 2026-07 快照)
- 方式: vendor 快照（已删 .git，代码随 robot_arm 仓库提交，Gerrit 不用 submodule）

## 位置与结构
放在 `robot_arm_driver/ros2_canopen/`。注意 `robot_arm_driver/` 是**容器目录**
（本身无 package.xml，colcon 才能扫到内部各包），ament 包本体在
`robot_arm_driver/arm_driver/`（包名仍为 robot_arm_driver，`$(find robot_arm_driver)`
等引用不受影响）。

## 保留的包（依赖链自洽）
lely_core_libraries → canopen_interfaces → canopen_core → canopen_base_driver
→ canopen_proxy_driver → canopen_402_driver → canopen_ros2_control；
canopen_master_driver（主站）、canopen_fake_slaves（vcan 假从站联调）。

## 已删除（非必要）
canopen（元包）、canopen_tests（示例）、canopen_utils（cogen，未用——我们走
generate_dcf/dcfgen）、canopen_ros2_controllers（专用控制器，上层用标准 JTC）、
Dockerfile/.github/.gitlab-ci.yml 等 CI 杂项。

## 升级方法
```bash
git clone -b humble https://github.com/ros-industrial/ros2_canopen.git /tmp/ros2_canopen
rm -rf /tmp/ros2_canopen/.git
# 只同步保留的包目录，再按上面清单删掉非必要包
# 更新本文件的提交号，整体编译回归后再提交
```

## 本地修改
无（纯上游快照，仅删包）。如需改动请记录在此，方便升级时重放。
