cmake_minimum_required(VERSION 3.5)
project(buoy_tests)

if(NOT BUILD_TESTING)
  return()
endif()

find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

# Linters
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

# GTest
find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
ament_find_gtest()

# Helper function to generate tests
function(buoy_add_test TEST_NAME)
  set(
    options
      ROS
      LAUNCH_TEST
  )
  set(oneValueArgs)
  set(multiValueArgs)

  cmake_parse_arguments(buoy_add_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  ament_add_gtest_executable(${TEST_NAME}
      tests/${TEST_NAME}.cpp
  )
  target_link_libraries(${TEST_NAME}
      ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
  )
  if(buoy_add_test_ROS)
    find_package(rclcpp REQUIRED)
    find_package(buoy_msgs REQUIRED)
    ament_target_dependencies(${TEST_NAME} rclcpp buoy_msgs)
  endif()
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  install(
    TARGETS ${TEST_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
  if(gz_add_plugin_LAUNCH_TEST)
    add_launch_test(launch/${TEST_NAME}.launch.py
      TIMEOUT 200
    )
  else()
    ament_add_gtest_test(${TEST_NAME})
  endif()

endfunction()

# Add tests
buoy_add_test(no_inputs)
buoy_add_test(no_inputs_ros_feedback ROS LAUNCH_TEST)

ament_package()
