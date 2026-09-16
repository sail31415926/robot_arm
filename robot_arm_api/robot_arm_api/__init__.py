# -*- coding: utf-8 -*-
"""
@file  __init__.py
@brief robot_arm_api 包入口：把 arm_commander_client 里的公开对象再导出一层，方便上层
       `from robot_arm_api import ArmApi, make_pose` 直接使用；reach_check（可达性 / 余量 / 能力卡）与
       llm_shot_loop 的核心闭环、拍摄接口规范 v9 适配层（shot_spec / shot_compiler / shot_executor 核心）
       都是纯 Python，不依赖 ROS——没有 rclpy 的环境（离线分析、单元测试）也能
       `from robot_arm_api import ArmModel, run_step_loop, ShotCompiler`。
"""

from .llm_shot_loop import (FEEDBACK_TOPIC, ManualClient, OpenAICompatClient, PromptBuilder,
                            ScriptedClient, StepRecord, fan_out, format_user_report,
                            make_feedback_publisher, run_llm_shot_loop, run_step_loop)
from .reach_check import (ArmModel, Margins, PlanReport, ReachResult, StepReport, arc_pose,
                          capability_card, capability_data, check_plan, check_pose,
                          current_state_text, headroom, headroom_schema, matrix_to_pose,
                          pose_from_look_at, pose_to_matrix)
from .reach_fit import RegionFit, fit_reach_region
from .shot_compiler import (DEFAULT_TOOL0_TO_OPTICAL, SPEED_PROFILES, CameraFrames, CompiledShot,
                            Primitive, ShotCompiler, WorldFrame)
from .shot_executor import (SHOT_FEEDBACK_TOPIC, ShotExecutor, ShotFeedback, ShotReport,
                            lookup_world_frame, make_shot_feedback_publisher, make_target_provider)
from .shot_spec import (ShotSpec, SpecError, SpecValidationError, TargetData, ValidationReport,
                        parse_shot, tracking_to_static, validate_shot, validate_target_data)

try:
    from .arm_commander_client import (HAS_GIMBAL_IFACE, ArmApi, ArmCommanderClient, CallResult,
                                       GimbalV2Client, execute_plan, make_pose, msg_to_yaml,
                                       offset_pose, pose_to_str, ros_type_name, run_plan_step,
                                       speed_code)
    HAS_ROS_CLIENT = True
except ImportError as _exc:  # 无 ROS 环境：只提供 reach_check 那一半
    HAS_ROS_CLIENT = False
    ROS_CLIENT_IMPORT_ERROR = _exc

__all__ = [
    # reach_check（纯 numpy）
    'ArmModel', 'Margins', 'ReachResult', 'StepReport', 'PlanReport', 'check_pose', 'check_plan',
    'headroom', 'headroom_schema', 'capability_card', 'capability_data', 'current_state_text',
    'pose_to_matrix', 'matrix_to_pose', 'pose_from_look_at', 'arc_pose', 'HAS_ROS_CLIENT',
    # reach_fit（可达区多项式拟合）
    'RegionFit', 'fit_reach_region',
    # 拍摄接口规范 v9 适配层：shot_spec（解析 / 校验 / 参考轨迹）→ shot_compiler（→ Commander 原语）→ shot_executor
    'ShotSpec', 'SpecError', 'SpecValidationError', 'ValidationReport', 'TargetData', 'validate_shot',
    'parse_shot', 'validate_target_data', 'tracking_to_static',
    'ShotCompiler', 'CompiledShot', 'Primitive', 'WorldFrame', 'CameraFrames', 'SPEED_PROFILES',
    'DEFAULT_TOOL0_TO_OPTICAL',
    'ShotExecutor', 'ShotReport', 'ShotFeedback', 'SHOT_FEEDBACK_TOPIC', 'make_shot_feedback_publisher',
    'make_target_provider', 'lookup_world_frame',
    # llm_shot_loop（核心闭环纯 Python；run_llm_shot_loop 运行时才要 ROS）
    'PromptBuilder', 'StepRecord', 'ScriptedClient', 'ManualClient', 'OpenAICompatClient',
    'run_step_loop', 'run_llm_shot_loop', 'format_user_report', 'FEEDBACK_TOPIC',
    'make_feedback_publisher', 'fan_out',
]
if HAS_ROS_CLIENT:
    __all__ += [
        'ArmCommanderClient', 'GimbalV2Client', 'ArmApi', 'CallResult', 'HAS_GIMBAL_IFACE',
        'make_pose', 'offset_pose', 'pose_to_str', 'speed_code', 'msg_to_yaml', 'ros_type_name',
        'execute_plan', 'run_plan_step',
    ]
