// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <OgreMesh.h>
#include <OgreMesh2.h>

//#include <ignition/rendering/ogre2/Ogre2Conversions.hh>

//#include <ignition/math/Matrix4.hh>
//#include <ignition/math/Vector3.hh>

namespace buoy
{
namespace mesh_utils
{
// Get the mesh information for the given mesh in v1 Ogre3D format. This is a really useful function that can be used by many
// different systems. e.g. physics mesh, navmesh, occlusion geometry etc...

// (Original) Code found on this forum link: http://www.ogre3d.org/wiki/index.php/RetrieveVertexData
// Edited Code (below) - (25/01/2016) By Jayce Young & Hannah Young at Aurasoft UK for the Skyline Game Engine.
void getV1MeshInformation(const Ogre::v1::MeshPtr mesh,
  size_t &vertex_count,
  Ogre::Vector3* &vertices,
  size_t &index_count,
  unsigned long* &indices,
  const Ogre::Vector3  &position,
  const Ogre::Quaternion  &orient,
  const Ogre::Vector3  &scale);

/*
  Get the mesh information for the given mesh in v2 Ogre3D format. This is a really useful function that can be used by many
  different systems. e.g. physics mesh, navmesh, occlusion geometry etc...

  Original Code - Code found on this forum link: http://www.ogre3d.org/wiki/index.php/RetrieveVertexData
  Most Code courtesy of al2950( thanks m8 :)), but then edited by Jayce Young & Hannah Young at Aurasoft UK (Skyline Game Engine) 
  to work with Items in the scene.
*/
void getMeshInformation(const Ogre::MeshPtr mesh,
  size_t &vertex_count,
  Ogre::Vector3* &vertices,
  size_t &index_count,
  Ogre::uint32* &indices,
  const Ogre::Vector3 &position,
  const Ogre::Quaternion &orient,
  const Ogre::Vector3 &scale);

}  // namespace mesh_utils
}  // namespace buoy

