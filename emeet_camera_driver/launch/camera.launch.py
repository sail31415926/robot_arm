import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('emeet_camera_driver')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    # 默认值面向独立启动；从 real.launch.py include 时会被覆写：
    #   disable_hid:=true publish_joint_states:=false
    # LaunchConfiguration 传给 Node parameters 时值是字符串 'true'/'false'，
    # rclcpp 会经过 YAML 解析自动转为 bool，无需额外转换。
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true',
    )
    disable_hid_arg = DeclareLaunchArgument(
        'disable_hid', default_value='false',
        description='true 时跳过 HID 初始化，仅做视频流（real.launch.py include 时需置 true）',
    )
    publish_joint_states_arg = DeclareLaunchArgument(
        'publish_joint_states', default_value='true',
        description='false 时不发 /joint_states（real.launch.py include 时由 joint_state_broadcaster 接管）',
    )

    return LaunchDescription([
        use_sim_time_arg,
        disable_hid_arg,
        publish_joint_states_arg,
        Node(
            package='emeet_camera_driver',
            executable='camera_driver_node',
            name='emeet_camera_node',
            output='screen',
            parameters=[params_file, {
                'use_sim_time':         LaunchConfiguration('use_sim_time'),
                'disable_hid':          LaunchConfiguration('disable_hid'),
                'publish_joint_states': LaunchConfiguration('publish_joint_states'),
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
            ],
        ),
    ])
