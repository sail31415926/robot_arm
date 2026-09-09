# -*- coding: utf-8 -*-
"""
@file  conftest.py
@brief reach_check 测试夹具：用 xacro 把 arm.urdf.xacro（含云台 V2）展开成完整 URDF 字符串。
       需要已 source ROS 与工作空间（xacro 模块 + robot_arm_description / robot_gimbal_description_v2
       的 share 目录）；任一缺失时整组测试 skip 而不是报错。
"""

import os
import sys

import pytest

# 允许不编译、直接在源码树里跑 pytest：把包根目录放进 sys.path
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

_WRAPPER = '''<robot name="eMeetArm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find robot_arm_description)/urdf/arm.urdf.xacro"/>
  <link name="world"/>
  <xacro:emeet_arm parent="world" xyz="0 0 0" rpy="0 0 0" sim_mode="false" backend="real"
                   arm_sim_mode="true" controllers_yaml=""/>
</robot>
'''


@pytest.fixture(scope='session')
def urdf_xml() -> str:
    """@brief 展开后的完整 URDF（arm_base_link → … → gimbal_tool0 → Cam0）。
    @return URDF XML 字符串；xacro / 描述包缺失时 skip
    """
    try:
        import xacro  # noqa: F401  (ROS 包，需 source)
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('robot_arm_description')
        get_package_share_directory('robot_gimbal_description_v2')
    except Exception as exc:  # pylint: disable=broad-except
        pytest.skip(f'需要 source ROS 与工作空间（xacro / 描述包）: {exc!r}')
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.xacro', delete=False) as fh:
        fh.write(_WRAPPER)
        path = fh.name
    try:
        return xacro.process_file(path).toxml()
    finally:
        os.unlink(path)
