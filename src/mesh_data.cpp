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

#include "mesh_data.h"

#include "base/perf.h"

void MeshData::CalculateNormals() {
  ScopedPerf _perf = ScopedPerf("Calculate normals");

  std::vector<Vertex>::iterator vi;

  // Start by clearing the normals.
  for (vi = vertices.begin(); vi != vertices.end(); vi++) {
    (*vi).normal = vec3(0);
  }

  // Calculate normals for all triangles, and add them to the triangle
  // vertices (i.e. each vertex's normal is the sum of the normals of all
  // neighbouring triangles).
  std::vector<Triangle>::iterator ti;
  for (ti = triangles.begin(); ti != triangles.end(); ti++) {
    // Vertices for this triangle.
    Vertex* v1 = &vertices[(*ti).a];
    Vertex* v2 = &vertices[(*ti).b];
    Vertex* v3 = &vertices[(*ti).c];

    // Calculate triangle normal (weighted by triangle area).
    // Note: Here we assume that the front side of the triangle is
    // counter-clockwise oriented.
    vec3 e1 = v2->position - v1->position;
    vec3 e2 = v3->position - v1->position;
    vec3 normal = e1.Cross(e2);

    // Add the normal to all the neighbouring vertices.
    v1->normal += normal;
    v2->normal += normal;
    v3->normal += normal;
  }

  // Normalize all the normals.
  for (vi = vertices.begin(); vi != vertices.end(); vi++) {
    (*vi).normal = (*vi).normal.Normalize();
  }

  _perf.Done();
}