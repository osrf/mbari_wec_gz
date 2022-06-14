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
ament_find_gtest()

# Helper function to generate tests
function(buoy_add_test TEST_NAME)

  ament_add_gtest(${TEST_NAME} tests/${TEST_NAME}.cpp)
  target_link_libraries(${TEST_NAME}
    ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
  )
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  install(
    TARGETS ${TEST_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

endfunction()

# Add tests
buoy_add_test(no_inputs)

ament_package()
