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

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
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
    EXTRA_ROS_PKGS
  )

  cmake_parse_arguments(gz_add_plugin "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${plugin_name} SHARED
    ${gz_add_plugin_SOURCES}
  )
  target_link_libraries(${plugin_name}
    PUBLIC
      gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
      gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
      gz-common${GZ_COMMON_VER}::profiler
      ${gz_add_plugin_PUBLIC_LINK_LIBS}
    PRIVATE
      ${gz_add_plugin_PRIVATE_LINK_LIBS}
  )
  if(gz_add_plugin_ROS)
    set(ROS_PKGS rclcpp buoy_interfaces)
    foreach(PKG ${ROS_PKGS})
      find_package(${PKG} REQUIRED)
    endforeach()
    ament_target_dependencies(${plugin_name} PUBLIC ${ROS_PKGS})
  endif()
  if(gz_add_plugin_EXTRA_ROS_PKGS)
    foreach(EXTRA_PKG ${gz_add_plugin_EXTRA_ROS_PKGS})
      find_package(${EXTRA_PKG} REQUIRED)
    endforeach()
    ament_target_dependencies(${plugin_name} PUBLIC ${gz_add_plugin_EXTRA_ROS_PKGS})
  endif()
  target_include_directories(${plugin_name}
      PUBLIC ${gz_add_plugin_INCLUDE_DIRS})
  # target_compile_features(${plugin_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  install(
    TARGETS ${plugin_name}
    DESTINATION lib)
endfunction()

add_subdirectory(src)

add_library(BuoyECMStateData INTERFACE)
target_include_directories(BuoyECMStateData INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS BuoyECMStateData
  EXPORT export_BuoyECMStateData
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/buoy_utils
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/ElectroHydraulicPTO
  DESTINATION include/buoy_gazebo
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/PolytropicPneumaticSpring
  DESTINATION include/buoy_gazebo
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/PTOFriction
  DESTINATION include/buoy_gazebo
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/MooringForce
  DESTINATION include/buoy_gazebo
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_targets(export_BuoyECMStateData)

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

endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
