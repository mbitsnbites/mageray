// -*- Mode: c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//------------------------------------------------------------------------------
// Copyright (c) 2013, Marcus Geelnard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------

#include "importers/openctm_importer.h"

#include <fstream>
#include <openctm2.h>

#include "base/log.h"
#include "base/perf.h"
#include "base/types.h"

static CTMuint MyCTMRead(void* buffer, CTMuint count, void* user_data) {
  std::istream* stream = static_cast<std::istream*>(user_data);
  ASSERT(stream, "No input stream found.");
  stream->read(reinterpret_cast<char*>(buffer), count);
  return static_cast<CTMuint>(stream->gcount());
}

bool OpenCTMImporter::Load(std::istream& stream) {
  ScopedPerf _perf = ScopedPerf("Load OpenCTM mesh");

#ifdef SCALAR_IS_FLOAT
  const CTMenum scalar_type = CTM_FLOAT;
#else
  const CTMenum scalar_type = CTM_DOUBLE;
#endif

  CTMenum err;

  // Create OpenCTM import context.
  CTMcontext ctm = ctmNewContext(CTM_IMPORT);
  err = ctmGetError(ctm);
  if (err != CTM_NONE) {
    LOG("Couldn't create OpenCTM import context: %s", ctmErrorString(err));
    return false;
  }

  // Read the file header.
  ctmOpenReadCustom(ctm, MyCTMRead, &stream);
  err = ctmGetError(ctm);
  if (err != CTM_NONE) {
    LOG("Couldn't open OpenCTM file: %s", ctmErrorString(err));
    ctmFreeContext(ctm);
    return false;
  }
  bool has_normals = ctmGetBoolean(ctm, CTM_HAS_NORMALS);
  bool has_uv_coords = ctmGetInteger(ctm, CTM_UV_MAP_COUNT) > 0;

  // Setup for reading.
  CTMuint num_triangles = ctmGetInteger(ctm, CTM_TRIANGLE_COUNT);
  CTMuint num_vertices = ctmGetInteger(ctm, CTM_VERTEX_COUNT);
  DLOG("%d triangles, %d vertices", num_triangles, num_vertices);
  m_data->triangles.resize(num_triangles);
  m_data->vertices.resize(num_vertices);
  ctmArrayPointer(ctm, CTM_INDICES, 3, CTM_UINT, 0, &m_data->triangles[0]);
  ctmArrayPointer(ctm, CTM_VERTICES, 3, scalar_type, sizeof(Vertex),
      &m_data->vertices[0].position);
  if (has_normals) {
    ctmArrayPointer(ctm, CTM_NORMALS, 3, scalar_type, sizeof(Vertex),
        &m_data->vertices[0].normal);
  }
  if (has_uv_coords) {
    ctmArrayPointer(ctm, CTM_UV_MAP_1, 2, scalar_type, sizeof(Vertex),
        &m_data->vertices[0].uv);
  }

  // Read mesh data.
  ctmReadMesh(ctm);
  err = ctmGetError(ctm);
  if (err != CTM_NONE) {
    LOG("Couldn't read OpenCTM file: %s", ctmErrorString(err));
    ctmFreeContext(ctm);
    return false;
  }

  // Free the OpenCTM context.
  ctmFreeContext(ctm);

  _perf.Done();

  // Generate normals?
  if (!has_normals) {
    m_data->CalculateNormals();
  }

  // Generate dummy UV coordinates?
  if (!has_uv_coords) {
    for (CTMuint i = 0; i < num_vertices; ++i) {
      vec3 pos = m_data->vertices[i].position;
      m_data->vertices[i].uv = vec2(pos.x, pos.y);
    }
  }

  return true;
}
