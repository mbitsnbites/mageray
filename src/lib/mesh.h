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

#include <istream>

#include "aabb.h"
#include "tree.h"
#include "mesh_data.h"

namespace mageray {

class Mesh {
  public:
    /// Load a mesh from a stream.
    /// @param stream The stream from which to read the mesh.
    /// @returns A Mesh object if the operation succeeded, or NULL.
    static Mesh* Load(std::istream& stream);

    /// Load a mesh from a file.
    /// @param file_name File to load.
    /// @returns A Mesh object if the operation succeeded, or NULL.
    static Mesh* Load(const char* file_name);

    /// Build a sphere mesh.
    /// @param res Sphere resolution.
    /// @param radius The sphere radius.
    /// @returns A Mesh object if the operation succeeded, or NULL.
    static Mesh* MakeSphere(int res, scalar radius);

    /// Build a plane mesh.
    /// @param size The plane size (the plane has the area size.u x size.v).
    /// @returns A Mesh object if the operation succeeded, or NULL.
    static Mesh* MakePlane(const vec2& size);

    /// Find intersection between mesh and ray.
    /// @param ray The ray to shoot into the tree.
    /// @param[in,out] hit Current closest hit information.
    /// @returns True if the ray intersects with the mesh.
    bool Intersect(const Ray& ray, HitInfo& hit) const {
      return m_tree->Intersect(ray, hit);
    }

    /// Calculate the normal for the given hit.
    /// @param hit[in,out] The HitInfo.
    void CompleteHitInfo(HitInfo& hit) const;

    /// @returns The bounding box for the mesh.
    const AABB& BoundingBox() const {
      return m_tree->BoundingBox();
    }

  private:
    Mesh(MeshData* data);

    /// The raw mesh data.
    const std::unique_ptr<const MeshData> m_data;

    /// The triangle tree (referencing the raw mesh data).
    const std::unique_ptr<const TriangleTree> m_tree;
};

} // namespace mageray

#endif // MAGERAY_MESH_H_
