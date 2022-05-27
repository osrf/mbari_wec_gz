find_package(ignition-math6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-math6_VERSION_MAJOR})

gz_add_plugin(PolytropicPneumaticSpring
  SOURCES
    PolytropicPneumaticSpring.cpp
  PUBLIC_LINK_LIBS
    ignition-math${IGN_RENDERING_VER}::ignition-math${IGN_RENDERING_VER}
)
