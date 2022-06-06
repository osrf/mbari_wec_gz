cmake_minimum_required(VERSION 3.5)
project(buoy_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(buoy_msgs REQUIRED)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})


# Plugins
#
# gz_add_plugin (<plugin_name>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>])
#              [INCLUDE_DIRS <paths_to_directories...>
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
      ignition-gazebo${GZ_GAZEBO_VER}::ignition-gazebo${GZ_GAZEBO_VER}
      ignition-plugin${GZ_PLUGIN_VER}::ignition-plugin${GZ_PLUGIN_VER}
      ${gz_add_plugin_PUBLIC_LINK_LIBS}
    PRIVATE
      ${gz_add_plugin_PRIVATE_LINK_LIBS}
  )
  if(gz_add_plugin_ROS)
    ament_target_dependencies(${plugin_name} PUBLIC rclcpp buoy_msgs)
  endif()
  target_include_directories(${plugin_name}
      PUBLIC ${gz_add_plugin_INCLUDE_DIRS})
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
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
