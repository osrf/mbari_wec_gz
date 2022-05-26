// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "MeshUtils.hpp"

#include <OgreSubMesh.h>
#include <OgreSubMesh2.h>
#include <Vao/OgreIndexBufferPacked.h>
#include <Vao/OgreAsyncTicket.h>
#include <OgreBitwise.h>

#include <ignition/common/Console.hh>

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
  const Ogre::Vector3  &scale)
{
  bool added_shared = false;
  size_t current_offset = 0;
  size_t shared_offset = 0;
  size_t next_offset = 0;
  size_t index_offset = 0;

  vertex_count = index_count = 0;

  // Calculate how many vertices and indices we're going to need
  for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::v1::SubMesh* submesh = mesh->getSubMesh(i);

    // We only need to add the shared vertices once
    if (submesh->useSharedVertices)
    {
      if (!added_shared)
      {
        vertex_count += mesh->sharedVertexData[0]->vertexCount;
        added_shared = true;
      }
    }
    else
    {
      vertex_count += submesh->vertexData[0]->vertexCount;
    }

    // Add the indices
    index_count += submesh->indexData[0]->indexCount;
  }

  // Allocate space for the vertices and indices
  vertices = new Ogre::Vector3[vertex_count];
  indices = new unsigned long[index_count];

  added_shared = false;

  // Run through the submeshes again, adding the data into the arrays
  for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::v1::SubMesh* submesh = mesh->getSubMesh(i);

    Ogre::v1::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData[0] : submesh->vertexData[0];

    if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
    {
      if (submesh->useSharedVertices)
      {
        added_shared = true;
        shared_offset = current_offset;
      }

      const Ogre::v1::VertexElement* posElem =
      vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

      Ogre::v1::HardwareVertexBufferSharedPtr vbuf =
      vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char* vertex =
      static_cast<unsigned char*>(vbuf->lock(Ogre::v1::HardwareBuffer::HBL_READ_ONLY));

      // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
      //  as second argument. So make it float, to avoid trouble when Ogre::Real will
      //  be comiled/typedefed as double:
      //  Ogre::Real* pReal;
      float* pReal;

      for (size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
      {
        posElem->baseVertexPointerToElement(vertex, &pReal);
        Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
        Ogre::Vector4 pt4(pReal[0], pReal[1], pReal[2], 0.0f);
        vertices[current_offset + j] = (orient * (pt * scale)) + position;
      }

      vbuf->unlock();
      next_offset += vertex_data->vertexCount;
    }


    Ogre::v1::IndexData* index_data = submesh->indexData[0];
    size_t numTris = index_data->indexCount / 3;
    Ogre::v1::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

    bool use32bitindexes = (ibuf->getType() == Ogre::v1::HardwareIndexBuffer::IT_32BIT);

    unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::v1::HardwareBuffer::HBL_READ_ONLY));
    unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


    size_t offset = (submesh->useSharedVertices) ? shared_offset : current_offset;

    if (use32bitindexes)
    {
      for (size_t k = 0; k < numTris * 3; ++k)
      {
        indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
      }
    }
    else
    {
      for (size_t k = 0; k < numTris * 3; ++k)
      {
        indices[index_offset++] = static_cast<unsigned long>(pShort[k]) +
          static_cast<unsigned long>(offset);
      }
    }

    ibuf->unlock();
    current_offset = next_offset;
  }
}

void getMeshInformation(const Ogre::MeshPtr mesh,
  size_t &vertex_count,
  Ogre::Vector3* &vertices,
  Ogre::Vector3* &normals,
  Ogre::Vector3* &textures,
  size_t &index_count,
  Ogre::uint32* &indices,
  const Ogre::Vector3 &position,
  const Ogre::Quaternion &orient,
  const Ogre::Vector3 &scale)
{
  //First, we compute the total number of vertices and indices and init the buffers.
  unsigned int numVertices = 0;
  unsigned int numIndices = 0;

  Ogre::Mesh::SubMeshVec::const_iterator subMeshIterator = mesh->getSubMeshes().begin();

  while (subMeshIterator != mesh->getSubMeshes().end())
  {
    Ogre::SubMesh *subMesh = *subMeshIterator;
    numVertices += subMesh->mVao[0][0]->getVertexBuffers()[0]->getNumElements();
    numIndices += subMesh->mVao[0][0]->getIndexBuffer()->getNumElements();

    subMeshIterator++;
  }

  vertices = new Ogre::Vector3[numVertices];
  normals = new Ogre::Vector3[numVertices];
  textures = new Ogre::Vector3[numVertices];
  indices = new Ogre::uint32[numIndices];

  vertex_count = numVertices;
  index_count = numIndices;

  unsigned int addedVertices = 0;
  unsigned int addedIndices = 0;

  unsigned int index_offset = 0;
  unsigned int subMeshOffset = 0;

  // Read Submeshes
  subMeshIterator = mesh->getSubMeshes().begin();
  while (subMeshIterator != mesh->getSubMeshes().end())
  {
    Ogre::SubMesh *subMesh = *subMeshIterator;
    Ogre::VertexArrayObjectArray vaos = subMesh->mVao[0];

    if (!vaos.empty())
    {
      //Get the first LOD level 
      Ogre::VertexArrayObject *vao = vaos[0];
      bool indices32 = (vao->getIndexBuffer()->getIndexType() == Ogre::IndexBufferPacked::IT_32BIT);

      const Ogre::VertexBufferPackedVec &vertexBuffers = vao->getVertexBuffers();
      Ogre::IndexBufferPacked *indexBuffer = vao->getIndexBuffer();

      //request async read from buffer 
      Ogre::VertexArrayObject::ReadRequestsArray requests;
      requests.push_back(Ogre::VertexArrayObject::ReadRequests(Ogre::VES_POSITION));

      vao->readRequests(requests);
      vao->mapAsyncTickets(requests);
      unsigned int subMeshVerticiesNum = requests[0].vertexBuffer->getNumElements();
      if (requests[0].type == Ogre::VET_HALF4)
      {
        ignerr << "VET_HALF4" << std::endl;
        for (size_t i = 0; i < subMeshVerticiesNum; ++i)
        {
          const Ogre::uint16* pos = reinterpret_cast<const Ogre::uint16*>(requests[0].data);
          Ogre::Vector3 p;
          Ogre::Vector3 n;
          Ogre::Vector3 tex;
          p.x = Ogre::Bitwise::halfToFloat(*pos++);
          p.y = Ogre::Bitwise::halfToFloat(*pos++);
          p.z = Ogre::Bitwise::halfToFloat(*pos++);
          n.x = Ogre::Bitwise::halfToFloat(*pos++);
          n.y = Ogre::Bitwise::halfToFloat(*pos++);
          n.z = Ogre::Bitwise::halfToFloat(*pos++);
          tex.x = Ogre::Bitwise::halfToFloat(*pos++);
          tex.y = Ogre::Bitwise::halfToFloat(*pos++);
          requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
          vertices[i + subMeshOffset] = (orient * (p * scale)) + position;
          normals[i + subMeshOffset] = (orient * (n * scale)) + position;
          textures[i + subMeshOffset] = tex;
        }
      }
      else if (requests[0].type == Ogre::VET_FLOAT3)
      {
        ignerr << "VET_FLOAT3" << std::endl;
        for (size_t i = 0; i < subMeshVerticiesNum; ++i)
        {
          const float* pos = reinterpret_cast<const float*>(requests[0].data);
          Ogre::Vector3 p;
          Ogre::Vector3 n;
          Ogre::Vector3 tex;
          p.x = *pos++;
          p.y = *pos++;
          p.z = *pos++;
          n.x = *pos++;
          n.y = *pos++;
          n.z = *pos++;
          tex.x = *pos++;
          tex.y = *pos++;
          requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
          vertices[i + subMeshOffset] = (orient * (p * scale)) + position;
          normals[i + subMeshOffset] = (orient * (n * scale)) + position;
          textures[i + subMeshOffset] = tex;
        }
      }
      else
      {
        ignerr << "Error: Vertex Buffer type not recognised in buoy::mesh_utils::::getMeshInformation" << std::endl;
      }
      subMeshOffset += subMeshVerticiesNum;
      vao->unmapAsyncTickets(requests);

      ////Read index data
      if (indexBuffer)
      {
        Ogre::AsyncTicketPtr asyncTicket = indexBuffer->readRequest(0, indexBuffer->getNumElements());

        unsigned int *pIndices = 0;
        if (indices32)
        {
          pIndices = (unsigned*)(asyncTicket->map());
        }
        else
        {
          unsigned short *pShortIndices = (unsigned short*)(asyncTicket->map());
          pIndices = new unsigned int[indexBuffer->getNumElements()];
          for (size_t k = 0; k < indexBuffer->getNumElements(); k++) pIndices[k] = static_cast<unsigned int>(pShortIndices[k]);
        }
        unsigned int bufferIndex = 0;

        for (size_t i = addedIndices; i < addedIndices + indexBuffer->getNumElements(); i++)
        {
          indices[i] = pIndices[bufferIndex] + index_offset;
          bufferIndex++;
        }
        addedIndices += indexBuffer->getNumElements();

        if (!indices32) delete[] pIndices;

        asyncTicket->unmap();
      }
      index_offset += vertexBuffers[0]->getNumElements();
    }
    subMeshIterator++;
  }
}

/*
void setMeshInformation(const Ogre::MeshPtr mesh,
  const size_t &vertex_count,
  const Ogre::Vector3* &vertices,
  const size_t &index_count,
  const Ogre::uint32* &indices,
  const Ogre::Vector3 &position,
  const Ogre::Quaternion &orient,
  const Ogre::Vector3 &scale)
{
  //First, we compute the total number of vertices and indices and init the buffers.
  unsigned int numVertices = 0;
  unsigned int numIndices = 0;

  Ogre::Mesh::SubMeshVec::const_iterator subMeshIterator = mesh->getSubMeshes().begin();

  while (subMeshIterator != mesh->getSubMeshes().end())
  {
    Ogre::SubMesh *subMesh = *subMeshIterator;
    numVertices += subMesh->mVao[0][0]->getVertexBuffers()[0]->getNumElements();
    numIndices += subMesh->mVao[0][0]->getIndexBuffer()->getNumElements();

    subMeshIterator++;
  }

  vertices = new Ogre::Vector3[numVertices];
  indices = new Ogre::uint32[numIndices];

  vertex_count = numVertices;
  index_count = numIndices;

  unsigned int addedVertices = 0;
  unsigned int addedIndices = 0;

  unsigned int index_offset = 0;
  unsigned int subMeshOffset = 0;

  // Read Submeshes
  subMeshIterator = mesh->getSubMeshes().begin();
  while (subMeshIterator != mesh->getSubMeshes().end())
  {
    Ogre::SubMesh *subMesh = *subMeshIterator;
    Ogre::VertexArrayObjectArray vaos = subMesh->mVao[0];

    if (!vaos.empty())
    {
      //Get the first LOD level 
      Ogre::VertexArrayObject *vao = vaos[0];
      bool indices32 = (vao->getIndexBuffer()->getIndexType() == Ogre::IndexBufferPacked::IT_32BIT);

      const Ogre::VertexBufferPackedVec &vertexBuffers = vao->getVertexBuffers();
      Ogre::IndexBufferPacked *indexBuffer = vao->getIndexBuffer();

      //request async read from buffer 
      Ogre::VertexArrayObject::ReadRequestsArray requests;
      requests.push_back(Ogre::VertexArrayObject::ReadRequests(Ogre::VES_POSITION));

      vao->readRequests(requests);
      vao->mapAsyncTickets(requests);
      unsigned int subMeshVerticiesNum = requests[0].vertexBuffer->getNumElements();
      if (requests[0].type == Ogre::VET_HALF4)
      {
        for (size_t i = 0; i < subMeshVerticiesNum; ++i)
        {
          const Ogre::uint16* pos = reinterpret_cast<const Ogre::uint16*>(requests[0].data);
          Ogre::Vector3 vec;
          vec.x = Ogre::Bitwise::halfToFloat(pos[0]);
          vec.y = Ogre::Bitwise::halfToFloat(pos[1]);
          vec.z = Ogre::Bitwise::halfToFloat(pos[2]);
          requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
          vertices[i + subMeshOffset] = (orient * (vec * scale)) + position;
        }
      }
      else if (requests[0].type == Ogre::VET_FLOAT3)
      {
        for (size_t i = 0; i < subMeshVerticiesNum; ++i)
        {
          const float* pos = reinterpret_cast<const float*>(requests[0].data);
          Ogre::Vector3 vec;
          vec.x = *pos++;
          vec.y = *pos++;
          vec.z = *pos++;
          requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
          vertices[i + subMeshOffset] = (orient * (vec * scale)) + position;
        }
      }
      else
      {
        ignerr << "Error: Vertex Buffer type not recognised in buoy::mesh_utils::getMeshInformation" << std::endl;
      }
      subMeshOffset += subMeshVerticiesNum;
      vao->unmapAsyncTickets(requests);

      ////Read index data
      if (indexBuffer)
      {
        Ogre::AsyncTicketPtr asyncTicket = indexBuffer->readRequest(0, indexBuffer->getNumElements());

        unsigned int *pIndices = 0;
        if (indices32)
        {
          pIndices = (unsigned*)(asyncTicket->map());
        }
        else
        {
          unsigned short *pShortIndices = (unsigned short*)(asyncTicket->map());
          pIndices = new unsigned int[indexBuffer->getNumElements()];
          for (size_t k = 0; k < indexBuffer->getNumElements(); k++) pIndices[k] = static_cast<unsigned int>(pShortIndices[k]);
        }
        unsigned int bufferIndex = 0;

        for (size_t i = addedIndices; i < addedIndices + indexBuffer->getNumElements(); i++)
        {
          indices[i] = pIndices[bufferIndex] + index_offset;
          bufferIndex++;
        }
        addedIndices += indexBuffer->getNumElements();

        if (!indices32) delete[] pIndices;

        asyncTicket->unmap();
      }
      index_offset += vertexBuffers[0]->getNumElements();
    }
    subMeshIterator++;
  }
}
*/
}  // namespace mesh_utils
}  // namespace buoy
