cmake_minimum_required(VERSION 3.5)
project(buoy_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GTest
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  # pytest/launch_testing
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Helper function to generate gtest
  function(buoy_add_gtest TEST_NAME)
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
    if(buoy_add_test_LAUNCH_TEST)
      add_launch_test(launch/${TEST_NAME}.launch.py
        TIMEOUT 200
      )
    else()
      ament_add_gtest_test(${TEST_NAME})
    endif()
  endfunction()

  # Helper function to generate pytest
  function(buoy_add_pytest PYTEST_NAME GTEST_NAME)
    find_package(rclpy REQUIRED)
    find_package(buoy_msgs REQUIRED)

    if(TARGET ${GTEST_NAME})
    else()
      ament_add_gtest_executable(${GTEST_NAME}
        tests/${GTEST_NAME}.cpp
      )

      target_link_libraries(${GTEST_NAME}
        ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
      )

      include_directories(${CMAKE_CURRENT_BINARY_DIR})
      install(
        TARGETS ${GTEST_NAME}
        DESTINATION lib/${PROJECT_NAME}
      )
    endif()

    # install(
    #   PROGRAMS tests/${PYTEST_NAME}.py
    #   DESTINATION lib/${PROJECT_NAME}
    # )

    add_launch_test(launch/${PYTEST_NAME}_py.launch.py
      TIMEOUT 200
    )
  endfunction()

  # Add gtests
  buoy_add_gtest(no_inputs)
  buoy_add_gtest(no_inputs_ros_feedback ROS LAUNCH_TEST)
  buoy_add_gtest(sc_commands_ros_feedback ROS LAUNCH_TEST)

  # Add pytests
  buoy_add_pytest(no_inputs no_inputs)
endif()

ament_package()
