cmake_minimum_required(VERSION 3.16)
project(robot_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmPose.msg"
  "msg/ArmTwist.msg"
  "msg/ArmFollowCommand.msg"
  "msg/ArmStatus.msg"
  "srv/ArmEnable.srv"
  "srv/ArmHoming.srv"
  "srv/ArmResetError.srv"
  "srv/ArmStop.srv"
  "action/ArmMoveToPose.action"
  "action/ArmTrajectoryShot.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs action_msgs)
ament_package()
