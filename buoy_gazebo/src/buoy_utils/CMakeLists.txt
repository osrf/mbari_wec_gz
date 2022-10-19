find_package(rclcpp REQUIRED)
add_library(Stopwatch
  SHARED
    Stopwatch.cpp
)
ament_target_dependencies(Stopwatch PUBLIC rclcpp)

install(
  TARGETS
    Stopwatch
  DESTINATION lib)

add_library(Interp1d SHARED Interp1d.cpp)
