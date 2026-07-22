cmake_minimum_required(VERSION 3.8)
project(titan_brain_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyObservation.msg"
  "msg/DirectionalSafetyObservation.msg"
  "msg/SafetyEvaluationStatus.msg"
  "msg/SafetyStabilityStatus.msg"
  "msg/EvaluatorObservabilityStatus.msg"
  "msg/SafetyIntent.msg"
  "msg/PermittedMotionEnvelope.msg"
  "msg/ArbitrationStatus.msg"
  "msg/CommandPathObservabilityStatus.msg"
  "msg/ActuatorFeedback.msg"
  "msg/StopAcknowledgement.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
