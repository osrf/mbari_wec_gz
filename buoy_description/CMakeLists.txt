cmake_minimum_required(VERSION 3.5)
project(buoy_description)
find_package(ament_cmake REQUIRED)

# Model Generation
set(BUOY_MODEL_PATH "/models/mbari_wec_base")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}${BUOY_MODEL_PATH})
add_custom_command(
  OUTPUT model_gen_cmd
  COMMAND empy3
    ${CMAKE_CURRENT_SOURCE_DIR}${BUOY_MODEL_PATH}/model.sdf.em >
    ${CMAKE_CURRENT_BINARY_DIR}${BUOY_MODEL_PATH}/model.sdf
)

add_custom_target(model_gen_target ALL
  DEPENDS model_gen_cmd
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  models
  ${CMAKE_CURRENT_BINARY_DIR}/models
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
