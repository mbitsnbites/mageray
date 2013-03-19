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

#ifndef MAGERAY_MESH_H_
#define MAGERAY_MESH_H_

#include <vector>

#include "aabb.h"
#include "tree.h"
#include "triangle.h"

class Mesh {
  public:
    Mesh() {}
    ~Mesh() {}

    /// Load a mesh from a file.
    /// @param file_name File to load.
    /// @returns true if the file could be loaded.
    bool Load(const char* file_name);

    /// Build a sphere mesh.
    /// @param res Sphere resolution.
    /// @param radius The sphere radius.
    void MakeSphere(int res, scalar radius);

    /// Find intersection between mesh and ray.
    /// @param ray The ray to shoot into the tree.
    /// @param[in,out] hit Current closest hit information.
    /// @returns True if the ray intersects with the mesh.
    bool Intersect(const Ray& ray, HitInfo& hit) {
      return m_tree.Intersect(ray, hit);
    }

    /// @returns The bounding box for the mesh.
    const AABB& BoundingBox() {
      return m_tree.BoundingBox();
    }

  private:
    /// Calculate the normals for the mesh.
    void CalculateNormals();

    std::vector<Triangle> m_triangles;
    std::vector<Vertex> m_vertices;
    TriangleTree m_tree;
};

#endif // MAGERAY_MESH_H_
