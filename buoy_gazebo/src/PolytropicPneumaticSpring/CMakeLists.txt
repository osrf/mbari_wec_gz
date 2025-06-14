find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
set(GZ_MATH_VER "")
#set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

gz_add_plugin(PolytropicPneumaticSpring
  SOURCES
    PolytropicPneumaticSpring.cpp
  INCLUDE_DIRS
    ..
    ../LatentData
  PUBLIC_LINK_LIBS
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)
