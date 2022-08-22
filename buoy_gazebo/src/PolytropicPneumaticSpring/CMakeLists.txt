find_package(ignition-math6 REQUIRED)
set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})

gz_add_plugin(PolytropicPneumaticSpring
  SOURCES
    PolytropicPneumaticSpring.cpp
  INCLUDE_DIRS
    ..
  PUBLIC_LINK_LIBS
    ignition-math${GZ_MATH_VER}::ignition-math${GZ_MATH_VER}
)
