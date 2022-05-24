set(OpenGL_GL_PREFERENCE "GLVND")
find_package(ignition-rendering6 REQUIRED COMPONENTS ogre)
set(IGN_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})
list(APPEND ign_ogre_components "Paging") #"RTShaderSystem" "Terrain" "Overlay" "Paging")
ign_find_package(IgnOGRE VERSION 1.9.0
  COMPONENTS ${ign_ogre_components})

find_package(ignition-math6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-math6_VERSION_MAJOR})

gz_add_plugin(TessendorfWaves
  SOURCES
    TessendorfWaves.cpp
    MeshUtils.cpp
  INCLUDE_DIRS
    ${OGRE_INCLUDE_DIRS}
  PUBLIC_LINK_LIBS
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}-ogre
    ignition-math${IGN_RENDERING_VER}::ignition-math${IGN_RENDERING_VER}
    ${OGRE_LIBRARIES}
)
