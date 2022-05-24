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

#include "MeshUtils.hpp"

namespace buoy
{
namespace mesh_utils
{
// code from here: https://wiki.ogre3d.org/RetrieveVertexData#Optimized_version
void getMeshInformation(const Ogre::Mesh *_mesh,
  size_t &_vertex_count,
  Ogre::Vector3* &_vertices,
  size_t &_index_count,
  uint64_t* &_indices,
  const ignition::math::Vector3d &_position,
  const ignition::math::Quaterniond &_orient,
  const ignition::math::Vector3d &_scale)
{
  bool added_shared = false;
  size_t current_offset = 0;
  size_t next_offset = 0;
  size_t index_offset = 0;

  _vertex_count = _index_count = 0;

  // Calculate how many vertices and indices we're going to need
  for (uint16_t i = 0; i < _mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = _mesh->getSubMesh(i);

    // We only need to add the shared vertices once
    if (submesh->useSharedVertices)
    {
      if (!added_shared)
      {
        _vertex_count += _mesh->sharedVertexData->vertexCount;
        added_shared = true;
      }
    }
    else
    {
      _vertex_count += submesh->vertexData->vertexCount;
    }

    // Add the indices
    _index_count += submesh->indexData->indexCount;
  }

  // Allocate space for the vertices and indices
  _vertices = new Ogre::Vector3[_vertex_count];
  _indices = new uint64_t[_index_count];

  added_shared = false;

  // Run through the submeshes again, adding the data into the arrays
  for (uint16_t i = 0; i < _mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = _mesh->getSubMesh(i);

    Ogre::VertexData* vertex_data = submesh->useSharedVertices ?
        _mesh->sharedVertexData : submesh->vertexData;

    if (!submesh->useSharedVertices || !added_shared)
    {
      if (submesh->useSharedVertices)
      {
        added_shared = true;
      }

      const Ogre::VertexElement* posElem =
        vertex_data->vertexDeclaration->findElementBySemantic(
            Ogre::VES_POSITION);

      Ogre::HardwareVertexBufferSharedPtr vbuf =
        vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char *vertex =
        static_cast<unsigned char*>(
            vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      // There is _no_ baseVertexPointerToElement() which takes an
      // Ogre::Real or a double as second argument. So make it float,
      // to avoid trouble when Ogre::Real will be comiled/typedefed as double:
      //      Ogre::Real* pReal;
      float *pReal;

      for (size_t j = 0; j < vertex_data->vertexCount;
           ++j, vertex += vbuf->getVertexSize())
     {
        posElem->baseVertexPointerToElement(vertex, &pReal);
        ignition::math::Vector3d pt(pReal[0], pReal[1], pReal[2]);
        _vertices[current_offset + j] =
            ignition::rendering::OgreConversions::Convert((_orient * (pt * _scale)) + _position);
      }

      vbuf->unlock();
      next_offset += vertex_data->vertexCount;
    }

    Ogre::IndexData* index_data = submesh->indexData;
    Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

    if ((ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT))
    {
      uint32_t*  pLong = static_cast<uint32_t*>(
          ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      for (size_t k = 0; k < index_data->indexCount; k++)
      {
        _indices[index_offset++] = pLong[k];
      }
    }
    else
    {
      uint64_t*  pLong = static_cast<uint64_t*>(
          ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      uint16_t* pShort = reinterpret_cast<uint16_t*>(pLong);
      for (size_t k = 0; k < index_data->indexCount; k++)
      {
        _indices[index_offset++] = static_cast<uint64_t>(pShort[k]);
      }
    }

    ibuf->unlock();
    current_offset = next_offset;
  }
}
}  // namespace mesh_utils
}  // namespace buoy
