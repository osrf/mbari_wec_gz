cmake_minimum_required(VERSION 3.5)
project(buoy_gazebo)

find_package(ament_cmake REQUIRED)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})


# Plugins
function(gz_add_plugin plugin_name)
  set(multiValueArgs
    SOURCES
    HEADERS
    PUBLIC_LINK_LIBS
    PRIVATE_LINK_LIBS
    INCLUDE_DIRS
  )

  cmake_parse_arguments(gz_add_plugin "${multiValueArgs}" ${ARGN})

  add_library(${plugin_name} SHARED
    ${gz_add_plugin_SOURCES}
  )
  target_link_libraries(${plugin_name}
    PUBLIC
      ${gz_add_plugin_PUBLIC_LINK_LIBS}
    PRIVATE
      ${gz_add_plugin_PRIVATE_LINK_LIBS}
  )
  target_include_directories(${plugin_name}
      PUBLIC ${gz_add_plugin_INCLUDE_DIRS})
  install (
    TARGETS ${plugin_name} 
    DESTINATION lib)
endfunction()


# Resources
install(DIRECTORY
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
