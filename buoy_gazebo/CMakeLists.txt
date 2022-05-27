cmake_minimum_required(VERSION 3.5)
project(buoy_gazebo)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Option to enable profiler
option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DGZ_PROFILER_ENABLE=1")
else()
  add_definitions("-DGZ_PROFILER_ENABLE=0")
endif()

find_package(ament_cmake REQUIRED)
find_package(buoy_description REQUIRED)
find_package(buoy_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

# Plugins
#
# gz_add_plugin (<plugin_name>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>])
#              [INCLUDE_DIRS <paths_to_directories...>
#              [ROS]
#
# Add a system plugin
#
# <plugin_name> Required. Name of the library
#
# [SOURCES]: Specify the source files for the plugin.
#
#
# [PUBLIC_LINK_LIBS]: Specify a list of libraries to be publicly linked.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
# [INCLUDE_DIRS]: Specify a list of path to directories to be included,
#
# [ROS]: Signal that this plugin is ROS-enabled

function(gz_add_plugin plugin_name)
  set(options ROS)
  set(oneValueArgs)
  set(multiValueArgs
    SOURCES
    PUBLIC_LINK_LIBS
    PRIVATE_LINK_LIBS
    INCLUDE_DIRS
  )

  cmake_parse_arguments(gz_add_plugin "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${plugin_name} SHARED
    ${gz_add_plugin_SOURCES}
  )
  target_link_libraries(${plugin_name}
    PUBLIC
      gz-common${GZ_COMMON_VER}::profiler
      gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
      gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
      gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
      gz-math${GZ_MATH_VER}::eigen3
      ${gz_add_plugin_PUBLIC_LINK_LIBS}
    PRIVATE
      ${gz_add_plugin_PRIVATE_LINK_LIBS}
  )
  if(gz_add_plugin_ROS)
    ament_target_dependencies(${plugin_name} PUBLIC rclcpp buoy_interfaces)
  endif()
  target_include_directories(${plugin_name}
      PUBLIC ${gz_add_plugin_INCLUDE_DIRS})
  # target_compile_features(${plugin_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  install(
    TARGETS ${plugin_name}
    DESTINATION lib)
endfunction()

add_subdirectory(src)


# Resources
install(DIRECTORY
  worlds
  launch
  gazebo
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GTest
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  # launch_testing
  find_package(launch_testing_ament_cmake REQUIRED)

  # Helper function to generate gtest
  function(gz_add_gtest TEST_NAME)
    set(
      options
        ROS
        LAUNCH_TEST
        GAZEBO
    )
    set(oneValueArgs)
    set(multiValueArgs
      EXTRA_ROS_PKGS)

    cmake_parse_arguments(gz_add_test "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    ament_add_gtest_executable(${TEST_NAME}
      test/${TEST_NAME}.cpp
    )
    if(gz_add_test_GAZEBO)
      target_link_libraries(${TEST_NAME}
        gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
      )
    endif()
    if(gz_add_test_ROS)
      set(ROS_PKGS rclcpp ${gz_add_test_EXTRA_ROS_PKGS})
      foreach(PKG ${ROS_PKGS})
        find_package(${PKG} REQUIRED)
      endforeach()
      ament_target_dependencies(${TEST_NAME} ${ROS_PKGS})
    endif()
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(${TEST_NAME}
      Stopwatch
    )
    install(
      TARGETS ${TEST_NAME}
      DESTINATION lib/${PROJECT_NAME}
    )
    if(gz_add_test_LAUNCH_TEST)
      add_launch_test(test/launch/${TEST_NAME}.launch.py
        TIMEOUT 300
      )
    else()
      ament_add_gtest_test(${TEST_NAME})
    endif()
  endfunction()

  # Add gtests
  gz_add_gtest(test_stopwatch ROS)

endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
