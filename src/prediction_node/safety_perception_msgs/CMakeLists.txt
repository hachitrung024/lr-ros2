cmake_minimum_required(VERSION 3.8)
project(safety_perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/TrajectoryStep.msg"
  "msg/Trajectory.msg"
  "msg/GeometryStep.msg"
  "msg/GeometryArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/RoverState.msg"
  "msg/ExternalWrench.msg"
  "msg/ExternalWrenchArray.msg"
  "msg/CollisionObject.msg"
  "msg/CollisionStep.msg"
  "msg/StabilityMomentEvidence.msg"
  "msg/ZmpEvidence.msg"
  "msg/RolloverStep.msg"
  "msg/PredictionOutput.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
