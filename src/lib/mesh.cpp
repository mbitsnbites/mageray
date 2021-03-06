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

#include <fstream>
#include <memory>

#include "base/log.h"
#include "base/perf.h"
#include "importers/openctm_importer.h"

namespace mageray {

Mesh::Mesh(MeshData* data) : m_data(data), m_tree(new TriangleTree(data)) {
}

Mesh* Mesh::Load(std::istream& stream) {
  std::unique_ptr<MeshData> data(new MeshData());

  // Try different file formats.
  bool success = false;
  if (OpenCTMImporter::Detect(stream)) {
    OpenCTMImporter importer(data.get());
    success = importer.Load(stream);
  }

  if (success) {
    return new Mesh(data.release());
  }

  LOG("Unable to load mesh file.");
  return NULL;
}

Mesh* Mesh::Load(const char* file_name) {
  std::ifstream is(file_name, std::ios_base::in | std::ios_base::binary);
  if (is.good()) {
    return Load(is);
  }

  LOG("Unable to open mesh file %s.", file_name);
  return NULL;
}

Mesh* Mesh::MakeSphere(int res, scalar radius) {
  ScopedPerf _perf = ScopedPerf(__FUNCTION__);

  std::unique_ptr<MeshData> data(new MeshData());

  // Make vertices
  int vertex_span = res + 1;
  int num_vertices = vertex_span * (res + 1);
  data->vertices.resize(num_vertices);
  scalar step = scalar(1.0) / static_cast<scalar>(res);
  scalar u = 0;
  for (int i = 0, k = 0; i <= res; ++i) {
    scalar cos_theta = std::cos(2*PI*u);
    scalar sin_theta = std::sin(2*PI*u);
    scalar v = 0;
    for (int j = 0; j <= res; ++j) {
      scalar cos_phi = std::cos(PI*v);
      scalar sin_phi = std::sin(PI*v);
      Vertex* vertex = &data->vertices[k++];
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
  data->triangles.resize(num_triangles);
  for (int i = 0, k = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      Triangle* triangle1 = &data->triangles[k++];
      Triangle* triangle2 = &data->triangles[k++];
      triangle1->a = triangle2->a = i * vertex_span + j;
      triangle1->b = i * vertex_span + j + 1;
      triangle1->c = triangle2->b = (i + 1) * vertex_span + j + 1;
      triangle2->c = (i + 1) * vertex_span + j;
    }
  }

  // Build triangle tree.
  Mesh* mesh = new Mesh(data.release());

  _perf.Done();

  return mesh;
}

Mesh* Mesh::MakePlane(const vec2& size) {
  ScopedPerf _perf = ScopedPerf(__FUNCTION__);

  std::unique_ptr<MeshData> data(new MeshData());

  // Make vertices.
  int num_vertices = 4;
  const vec2 sides = size * scalar(0.5);
  data->vertices.resize(num_vertices);
  data->vertices[0].normal = vec3(0.0, 0.0, 1.0);
  data->vertices[0].position = vec3(-sides.u, -sides.v, scalar(0.0));
  data->vertices[0].uv = vec2(0.0, 0.0);
  data->vertices[1].normal = vec3(0.0, 0.0, 1.0);
  data->vertices[1].position = vec3(sides.u, -sides.v, scalar(0.0));
  data->vertices[1].uv = vec2(1.0, 0.0);
  data->vertices[2].normal = vec3(0.0, 0.0, 1.0);
  data->vertices[2].position = vec3(-sides.u, sides.v, scalar(0.0));
  data->vertices[2].uv = vec2(0.0, 1.0);
  data->vertices[3].normal = vec3(0.0, 0.0, 1.0);
  data->vertices[3].position = vec3(sides.u, sides.v, scalar(0.0));
  data->vertices[3].uv = vec2(1.0, 1.0);

  // Make triangles.
  int num_triangles = 2;
  data->triangles.resize(num_triangles);
  data->triangles[0].a = data->triangles[1].a = 0;
  data->triangles[0].b = 1;
  data->triangles[0].c = data->triangles[1].b = 3;
  data->triangles[1].c = 2;

  // Build triangle tree.
  Mesh* mesh = new Mesh(data.release());

  _perf.Done();

  return mesh;
}

void Mesh::CompleteHitInfo(HitInfo& hit) const {
  // Pick the three triangle vertices.
  const Vertex* v1 = &m_data->vertices[hit.triangle->a];
  const Vertex* v2 = &m_data->vertices[hit.triangle->b];
  const Vertex* v3 = &m_data->vertices[hit.triangle->c];

  // Interpolation weighting factors.
  scalar w1 = scalar(1.0) - hit.tri_uv.u - hit.tri_uv.v;
  scalar w2 = hit.tri_uv.u;
  scalar w3 = hit.tri_uv.v;

  // Interpolate and normalize normal.
  hit.normal =
      (v1->normal * w1 + v2->normal * w2 + v3->normal * w3).Normalize();

  // Interpolate U/V coordinates.
  hit.uv = v1->uv * w1 + v2->uv * w2 + v3->uv * w3;
}

} // namespace mageray
