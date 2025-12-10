cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息文件
set(msg_files
  "msg/MotorCtrlNormal.msg"
  "msg/MotorCtrltoPos.msg"
  "msg/ArmCtrl.msg"
  "msg/BunkerBmsStatus.msg"
  "msg/BunkerMotorState.msg"
  "msg/BunkerRsStatus.msg"
  "msg/BunkerStatus.msg"
  "msg/LiftCtrl.msg"
  "msg/ExtendDevCtrl.msg"
  "msg/LightCtrl.msg"
  "msg/Robot2TurnoutRequest.msg"
  "msg/HeartbeatBag.msg"
  "msg/InspectTaskInfoSimple.msg"
  "msg/InspectTaskPlan.msg"
  "msg/InspectTaskInstance.msg"
  "msg/InspectTaskExecutionStart.msg"
  "msg/InspectTaskExecutionComplete.msg"
  "msg/MonitorImgDataCollectCmd.msg"
  "msg/MonitorImgDataCollectSend.msg"
  "msg/MonitorTaskDataUpload.msg"
  "msg/Arm4JointPos.msg"
  "msg/RobotRuntimePosition.msg"
  "msg/RobotBatteryStatus.msg"
  "msg/TrackRobotRuntimeMoveStatus.msg"
)

# 服务文件
set(srv_files
  "srv/ArmPosQuery.srv"
  "srv/ArmCtrlSrv.srv"
  "srv/AvoidObstaclesPauseMove.srv"
  "srv/AvoidObstaclesPauseTask.srv"
  "srv/AvoidObstaclesStartMove.srv"
  "srv/AvoidObstaclesStartTask.srv"
  "srv/BatteryChargeExecuteProgram.srv"
  "srv/BatteryChargeStartTask.srv"
  "srv/BatteryChargeNav.srv"
  "srv/ChangeCtrlModeCmd.srv"
  "srv/BatteryChargePauseTask.srv"
  "srv/CtrlModeQuery.srv"
  "srv/ExtendDevCtrlSrv.srv"
  "srv/InspectTaskInfoListQuery.srv"
  "srv/InspectTaskInfoDetailQuery.srv"
  "srv/InspectTaskInfoConfigureCmd.srv"
  "srv/InspectTaskInfoDeleteCmd.srv"
  "srv/InspectTaskPlanListQuery.srv"
  "srv/InspectTaskPlanAddCmd.srv"
  "srv/InspectTaskPlanDeleteCmd.srv"
  "srv/InspectTaskCtrlCmd.srv"
  "srv/InspectTaskInstanceQuery.srv"
  "srv/LiftCtrlSrv.srv"
  "srv/LiftPosQuery.srv"
  "srv/MonitorSensorDataCollectCmd.srv"
  "srv/MonitorCollectDataResend.srv"
  "srv/MonitorTaskDataReupload.srv"
  "srv/MotorCtrltoPosSrv.srv"
  "srv/ModeChangePauseTask.srv"
  "srv/ModeChangeStartTask.srv"
  "srv/RobotInfoQuery.srv"
  "srv/SystemTimeSyncCmd.srv"
  "srv/TaskStartOrFinishResponse.srv"
  "srv/TaskDataCollectSrv.srv"
  "srv/TaskExecuteStatusQuery.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs 
)

ament_export_dependencies(rosidl_default_runtime)



ament_package()
