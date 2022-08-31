cmake_minimum_required(VERSION 3.8)
project(splinter)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_SOURCE_DIR}/splinter)

# lint_cmake: -readability/wonkycase
ExternalProject_Add(splinter
    GIT_REPOSITORY https://github.com/bgrimstad/splinter
    GIT_TAG 2c877c0d7f68025764f7af048cc609e5c05080e0  # v3.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
)

add_custom_target(touch_ignore_splinter ALL
                  COMMAND ${CMAKE_COMMAND} -E touch
                  ${EXTERNAL_INSTALL_LOCATION}/AMENT_IGNORE)

add_custom_target(mkdir_include ALL
                  COMMAND ${CMAKE_COMMAND} -E make_directory
                  ${CMAKE_SOURCE_DIR}/include)

add_custom_target(touch_ignore_include ALL
                  COMMAND ${CMAKE_COMMAND} -E touch
                  ${CMAKE_SOURCE_DIR}/include/AMENT_IGNORE)
add_dependencies(touch_ignore_include mkdir_include)

add_custom_target(link_target_includes ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink
                  ${EXTERNAL_INSTALL_LOCATION}/include/SPLINTER
                  ${CMAKE_SOURCE_DIR}/include/splinter)
add_dependencies(link_target_includes touch_ignore_include)

add_custom_target(link_target_libs ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink
                  ${EXTERNAL_INSTALL_LOCATION}/lib
                  ${CMAKE_SOURCE_DIR}/lib)

install(FILES lib/libsplinter-3-0.so
  DESTINATION lib)

install(
  DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include/SPLINTER/
  DESTINATION include/splinter
)

ament_export_include_directories(include)
ament_export_libraries(lib/libsplinter-3-0.so)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
