cmake_minimum_required(VERSION 3.5)
project(buoy_tests)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunServer.srv"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

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
    set(multiValueArgs
      EXTRA_ROS_PKGS)

    cmake_parse_arguments(buoy_add_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    ament_add_gtest_executable(${TEST_NAME}
      tests/${TEST_NAME}.cpp
    )
    target_link_libraries(${TEST_NAME}
      gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    )
    if(buoy_add_test_ROS)
      set(ROS_PKGS rclcpp buoy_interfaces buoy_api_cpp ${buoy_add_test_EXTRA_ROS_PKGS})
      foreach(PKG ${ROS_PKGS})
        find_package(${PKG} REQUIRED)
      endforeach()
      ament_target_dependencies(${TEST_NAME} ${ROS_PKGS})
    endif()
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    install(
      TARGETS ${TEST_NAME}
      DESTINATION lib/${PROJECT_NAME}
    )
    if(buoy_add_test_LAUNCH_TEST)
      add_launch_test(launch/${TEST_NAME}.launch.py
        TIMEOUT 300
      )
    else()
      ament_add_gtest_test(${TEST_NAME})
    endif()
  endfunction()

  # Helper function to generate pytest
  function(buoy_add_pytest PYTEST_NAME #[[optional: GTEST_NAME]])
    find_package(buoy_api_cpp REQUIRED)
    find_package(buoy_interfaces REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclpy REQUIRED)

    if(DEFINED ARGV1)
      set(GTEST_NAME ${ARGV1})
      if(TARGET ${GTEST_NAME})
        # pass
      else()
        ament_add_gtest_executable(${GTEST_NAME}
          tests/${GTEST_NAME}.cpp
        )

        target_link_libraries(${GTEST_NAME}
          gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
        )
        rosidl_target_interfaces(${GTEST_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
        ament_target_dependencies(${GTEST_NAME} rclcpp)

        include_directories(${CMAKE_CURRENT_BINARY_DIR})
        install(
          TARGETS ${GTEST_NAME}
          DESTINATION lib/${PROJECT_NAME}
        )
      endif()
    endif()

    add_launch_test(launch/${PYTEST_NAME}_py.launch.py
      TIMEOUT 300
    )
  endfunction()

  ament_python_install_package(testing_utils)

  # Add gtests
  buoy_add_gtest(no_inputs)
  buoy_add_gtest(no_inputs_ros_feedback ROS LAUNCH_TEST)
  buoy_add_gtest(pc_commands_ros_feedback ROS LAUNCH_TEST)
  buoy_add_gtest(sc_commands_ros_feedback ROS LAUNCH_TEST)

  # Add pytests
  buoy_add_pytest(no_inputs no_inputs)
  buoy_add_pytest(pc_bias_damping_ros_feedback fixture_server_sinusoidal_piston)
  buoy_add_pytest(pc_commands_ros_feedback fixture_server)
  buoy_add_pytest(sc_pump_ros_feedback fixture_server)
  buoy_add_pytest(sc_valve_ros_feedback fixture_server)

  # Note: This is an xfail test until fixed upstream
  ament_add_pytest_test(test_fixture tests/test_fixture.py)

  install(PROGRAMS
    tests/test_fixture.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
