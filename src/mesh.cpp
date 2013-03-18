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

#include "mesh.h"

#include <openctm2.h>

#include "base/log.h"
#include "base/perf.h"

void Mesh::MakeSphere(int res, scalar radius) {
  ScopedPerf _perf = ScopedPerf(__FUNCTION__);

  // Make vertices
  int vertex_span = res + 1;
  int num_vertices = vertex_span * (res + 1);
  m_vertices.resize(num_vertices);
  scalar step = 1.0 / static_cast<scalar>(res);
  scalar u = 0;
  for (int i = 0, k = 0; i <= res; ++i) {
    scalar cos_theta = std::cos(2*PI*u);
    scalar sin_theta = std::sin(2*PI*u);
    scalar v = 0;
    for (int j = 0; j <= res; ++j) {
      scalar cos_phi = std::cos(PI*v);
      scalar sin_phi = std::sin(PI*v);
      Vertex* vertex = &m_vertices[k++];
      vertex->normal = vec3(
        cos_theta * sin_phi,
        sin_theta * sin_phi,
        cos_phi
      );
      vertex->position = vertex->normal * radius;
      vertex->uv = vec2(u, v);
      v += step;
    }
    u += step;
  }

  // Make triangles
  int num_triangles = res * res * 2;
  m_triangles.resize(num_triangles);
  for (int i = 0, k = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      Triangle* triangle1 = &m_triangles[k++];
      Triangle* triangle2 = &m_triangles[k++];
      triangle1->a = triangle2->a = i * vertex_span + j;
      triangle1->b = i * vertex_span + j + 1;
      triangle1->c = triangle2->b = (i + 1) * vertex_span + j + 1;
      triangle2->c = (i + 1) * vertex_span + j;
    }
  }

  _perf.Done();
}

bool Mesh::Load(const char* file_name) {
  ScopedPerf _perf = ScopedPerf("Load mesh");

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
  ctmOpenReadFile(ctm, file_name);
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
  m_triangles.resize(num_triangles);
  m_vertices.resize(num_vertices);
  ctmArrayPointer(ctm, CTM_INDICES, 3, CTM_UINT, 0, &m_triangles[0]);
  ctmArrayPointer(ctm, CTM_VERTICES, 3, scalar_type, sizeof(Vertex), &m_vertices[0].position);
  if (has_normals) {
    ctmArrayPointer(ctm, CTM_NORMALS, 3, scalar_type, sizeof(Vertex), &m_vertices[0].normal);
  }
  if (has_uv_coords) {
    ctmArrayPointer(ctm, CTM_UV_MAP_1, 2, scalar_type, sizeof(Vertex), &m_vertices[0].uv);
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

  // Generate normals?
  if (!has_normals) {
    // TODO(mage): Calculate normals...
    for (CTMuint i = 0; i < num_vertices; ++i) {
      m_vertices[i].normal = vec3(0, 0, 1);
    }
  }

  // Generate dummy UV coordinates?
  if (!has_uv_coords) {
    for (CTMuint i = 0; i < num_vertices; ++i) {
      vec3 pos = m_vertices[i].position;
      m_vertices[i].uv = vec2(pos.x, pos.y);
    }
  }

  _perf.Done();

  // Build triangle tree.
  m_tree.Build(m_triangles, m_vertices);

  return true;
}
