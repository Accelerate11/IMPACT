cmake_minimum_required(VERSION 3.8)
project(xq_sim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentState.msg"
  "msg/FaultEvent.msg"
  "msg/FrontierBid.msg"
  "msg/HealthStatus.msg"
  "msg/LocalizationQuality.msg"
  "msg/LocalizationGeometry.msg"
  "msg/DirectionalIntegrity.msg"
  "msg/AlertLimit.msg"
  "msg/AlertLimitProfile.msg"
  "msg/IntegrityMargin.msg"
  "msg/InformationMap.msg"
  "msg/ActivePerceptionDecision.msg"
  "msg/ReplanEvent.msg"
  "msg/SubmapMeta.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
