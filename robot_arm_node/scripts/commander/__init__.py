#!/usr/bin/env python3
"""
@file   __init__.py
@brief  Arm Commander 中间层包
@version 1.0
@date   2026-06-09

Arm Commander 对外暴露：
  - ArmCommanderNode：主节点，可 headless 运行或导入到 GUI
  - MoveToPoseServer：ArmMoveToPose Action Server 逻辑
  - MotionExecutor：共享运动执行引擎
  - StatusAggregator：状态聚合与 ArmStatus 发布

分层关系：
  Director (上层)  ←→  Arm Commander (本包)  ←→  Driver (下层, robot_arm_driver)

@copyright Copyright (c) 2026 eMeet
"""

"""
Arm Commander 中间层 —— 可 import 的模块包。

注意：ArmCommanderNode 主节点位于本包外部（lib/robot_arm_node/arm_commander_node.py），
通过绝对导入使用本包：
    from commander.move_to_pose_server import MoveToPoseServer
    from commander.motion_executor import MotionExecutor
    from commander.status_aggregator import StatusAggregator

用例：
    from commander import MoveToPoseServer, MotionExecutor, StatusAggregator
"""

from .move_to_pose_server import MoveToPoseServer
from .trajectory_shot_server import TrajectoryShotServer
from .motion_executor import MotionExecutor
from .status_aggregator import StatusAggregator

__all__ = ['MoveToPoseServer', 'TrajectoryShotServer', 'MotionExecutor', 'StatusAggregator']
