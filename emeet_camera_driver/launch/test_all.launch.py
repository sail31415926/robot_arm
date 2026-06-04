from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动相机驱动节点 (集成 UVC 视频流与 HID 控制)
        Node(
            package='emeet_camera_driver',
            executable='camera_driver_node',
            name='emeet_camera_node',
            output='screen',
            parameters=[{
                'camera_type': 'auto',
                'width': 1920,
                'height': 1080,
                'fps': 30,
                'pixel_format': 'MJPEG',
                'publish_display_raw': False,
                'publish_display_compressed': True,
            }],
            # 关键：驱动发布的是私有话题 ~/image_raw（实际为 /emeet_camera_node/image_raw）
            # UI/算法侧订阅统一使用 /camera/image_raw
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
            ],
        ),
        
        # 2. 启动测试 UI 界面
        Node(
            package='emeet_camera_driver',
            executable='camera_test_gui.py',
            name='emeet_camera_gui',
            output='screen'
        )
    ])
