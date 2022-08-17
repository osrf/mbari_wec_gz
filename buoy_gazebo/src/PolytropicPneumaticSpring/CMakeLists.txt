find_package(gz-math6 REQUIRED)
set(GZ_MATH_VER ${gz-math6_VERSION_MAJOR})

gz_add_plugin(PolytropicPneumaticSpring
  SOURCES
    PolytropicPneumaticSpring.cpp
  INCLUDE_DIRS
    ..
  PUBLIC_LINK_LIBS
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)
