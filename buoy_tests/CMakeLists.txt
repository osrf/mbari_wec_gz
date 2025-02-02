cmake_minimum_required(VERSION 3.5)
project(buoy_tests)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunServer.srv"
  ADD_LINTER_TESTS
)

find_package(buoy_gazebo REQUIRED)
message(STATUS ${buoy_gazebo_INCLUDE_DIRS})

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
      EXTRA_ROS_PKGS
      LOCALLY_INSTALLED_LIBS
      BOOST_COMPONENTS)

    cmake_parse_arguments(buoy_add_gtest "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(buoy_add_gtest_BOOST_COMPONENTS)
      set(Boost_USE_STATIC_LIBS OFF)
      set(Boost_USE_MULTITHREADED ON)
      set(Boost_USE_STATIC_RUNTIME OFF)
      find_package(Boost 1.71.0 REQUIRED COMPONENTS ${buoy_add_gtest_BOOST_COMPONENTS})
      set(BOOST_LIBS "")
      foreach(BOOST_COMP ${buoy_add_gtest_BOOST_COMPONENTS})
        list(APPEND BOOST_LIBS "Boost::${BOOST_COMP}")
      endforeach()
    endif()

    if(buoy_add_gtest_LOCALLY_INSTALLED_LIBS)
      foreach(LOCAL_LIB ${buoy_add_gtest_LOCALLY_INSTALLED_LIBS})
        find_package(${LOCAL_LIB} REQUIRED)
      endforeach()
    endif()

    ament_add_gtest_executable(${TEST_NAME}
      tests/${TEST_NAME}.cpp
    )

    target_link_libraries(${TEST_NAME}
      gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    )

    if(buoy_add_gtest_LOCALLY_INSTALLED_LIBS)
      foreach(LOCAL_LIB ${buoy_add_gtest_LOCALLY_INSTALLED_LIBS})
        target_link_libraries(${TEST_NAME} ${LOCAL_LIB})
      endforeach()
    endif()

    if(buoy_add_gtest_BOOST_COMPONENTS)
      target_link_libraries(${TEST_NAME}
        ${BOOST_LIBS}
      )
    endif()

    if(buoy_add_gtest_ROS)
      set(ROS_PKGS rclcpp buoy_interfaces buoy_api_cpp)
      foreach(PKG ${ROS_PKGS})
        find_package(${PKG} REQUIRED)
      endforeach()
      ament_target_dependencies(${TEST_NAME} ${ROS_PKGS})
    endif()
    if(buoy_add_gtest_EXTRA_ROS_PKGS)
      foreach(EXTRA_PKG ${buoy_add_gtest_EXTRA_ROS_PKGS})
        find_package(${EXTRA_PKG} REQUIRED)
      endforeach()
      ament_target_dependencies(${TEST_NAME} ${buoy_add_gtest_EXTRA_ROS_PKGS})
    endif()

    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    install(
      TARGETS ${TEST_NAME}
      DESTINATION lib/${PROJECT_NAME}
    )

    if(buoy_add_gtest_LAUNCH_TEST)
      add_launch_test(launch/${TEST_NAME}.launch.py
        TIMEOUT 1000
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
        rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
        target_link_libraries(${GTEST_NAME} "${cpp_typesupport_target}")
        ament_target_dependencies(${GTEST_NAME} rclcpp)

        include_directories(${CMAKE_CURRENT_BINARY_DIR})
        install(
          TARGETS ${GTEST_NAME}
          DESTINATION lib/${PROJECT_NAME}
        )
      endif()
    endif()

    add_launch_test(launch/${PYTEST_NAME}_py.launch.py
      TIMEOUT 1000
    )
  endfunction()

  ament_python_install_package(testing_utils)

  # Add gtests
  buoy_add_gtest(experiment_comparison ROS LAUNCH_TEST
    BOOST_COMPONENTS
      iostreams system filesystem
    EXTRA_ROS_PKGS
      buoy_gazebo
      simple_interp)

  buoy_add_gtest(eh_solver ROS LAUNCH_TEST
    BOOST_COMPONENTS
      iostreams system filesystem
    EXTRA_ROS_PKGS
      buoy_gazebo)

  buoy_add_gtest(eh_windtarget ROS LAUNCH_TEST
    BOOST_COMPONENTS
      iostreams system filesystem
    EXTRA_ROS_PKGS
      buoy_gazebo)

  buoy_add_gtest(wave_body_interactions_heaveonly LAUNCH_TEST
    LOCALLY_INSTALLED_LIBS
      FreeSurfaceHydrodynamics
    EXTRA_ROS_PKGS
      ament_index_cpp)

  buoy_add_gtest(wave_body_interactions_pitchonly LAUNCH_TEST
    LOCALLY_INSTALLED_LIBS
      FreeSurfaceHydrodynamics
    EXTRA_ROS_PKGS
      ament_index_cpp)

  buoy_add_gtest(mooring_force)
  set_tests_properties(mooring_force PROPERTIES TIMEOUT 1000)

  buoy_add_gtest(no_inputs_ros_feedback ROS LAUNCH_TEST)
  buoy_add_gtest(pc_commands_ros_feedback ROS LAUNCH_TEST)
  buoy_add_gtest(sc_commands_ros_feedback ROS LAUNCH_TEST)

  buoy_add_gtest(test_pto_friction ROS
    EXTRA_ROS_PKGS
      simple_interp)

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
    worlds
    DESTINATION share/${PROJECT_NAME}
  )

  install(CODE "execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/test_inputdata
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test_inputdata)"
  )

  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

endif()


ament_package()
