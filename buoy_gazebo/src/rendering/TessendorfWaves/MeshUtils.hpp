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
#include <OgreSubMesh.h>

#include <ignition/rendering/ogre/OgreConversions.hh>

#include <ignition/math/Matrix4.hh>
#include <ignition/math/Vector3.hh>

namespace buoy
{
namespace mesh_utils
{
// code found here: https://wiki.ogre3d.org/RetrieveVertexData#Optimized_version
void getMeshInformation(const Ogre::Mesh *_mesh,
  size_t &_vertex_count,
  Ogre::Vector3* &_vertices,
  size_t &_index_count,
  uint64_t* &_indices,
  const ignition::math::Vector3d &_position,
  const ignition::math::Quaterniond &_orient,
  const ignition::math::Vector3d &_scale);
}  // namespace mesh_utils
}  // namespace buoy

