find_package(rclcpp REQUIRED)
add_library(StopwatchSimTime
  SHARED
    StopwatchSimTime.cpp
)
ament_target_dependencies(StopwatchSimTime PUBLIC rclcpp)

install(
  TARGETS
    StopwatchSimTime
  DESTINATION lib)
