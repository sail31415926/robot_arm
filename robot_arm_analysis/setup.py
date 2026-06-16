from setuptools import find_packages, setup

package_name = 'robot_arm_analysis'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', [
            'scripts/plot_ee_velocity',
            'scripts/plot_joint_torques',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eMeet',
    maintainer_email='sw.claude.Ethan@emeet.com',
    description='机械臂运动数据采集与分析',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'record_motion_data = arm_analysis.data_recorder:main',
            'live_plot = arm_analysis.live_plot_node:main',
            'joint_states_monitor = arm_analysis.joint_states_monitor:main',
        ],
    },
)
