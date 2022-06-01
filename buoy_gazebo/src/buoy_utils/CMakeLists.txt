add_library(StopwatchSimTime
  SHARED
    StopwatchSimTime.cc
)

ament_target_dependencies(StopwatchSimTime PUBLIC rclcpp)

install(
  TARGETS StopwatchSimTime
  DESTINATION lib)
