set(OpenGL_GL_PREFERENCE "GLVND")
find_package(ignition-rendering6 REQUIRED COMPONENTS ogre2)
set(IGN_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})
ign_find_package(IgnOGRE2 VERSION 2.2.0)

find_package(ignition-math6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-math6_VERSION_MAJOR})

gz_add_plugin(TessendorfWaves
  SOURCES
    TessendorfWaves.cpp
    MeshUtils.cpp
  INCLUDE_DIRS
    ${OGRE2_INCLUDE_DIRS}
  PUBLIC_LINK_LIBS
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}-ogre2
    ignition-math${IGN_RENDERING_VER}::ignition-math${IGN_RENDERING_VER}
    ${OGRE2_LIBRARIES}
)
