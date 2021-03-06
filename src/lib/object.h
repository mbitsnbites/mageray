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

#ifndef MAGERAY_OBJECT_H_
#define MAGERAY_OBJECT_H_

#include "base/log.h"
#include "base/types.h"
#include "mat.h"
#include "material.h"
#include "mesh.h"

namespace mageray {

class Object {
  public:
    Object(const Mesh* mesh);
    ~Object() {}

    /// Translate the object.
    /// @param t Translation.
    void Translate(const vec3& t);

    /// Scale the object.
    /// @param s Scale.
    void Scale(const vec3& s);

    /// Rotate the object.
    /// @param r Rotation around x, y and z (degrees, in that order).
    void Rotate(const vec3& r);

    /// Set the transformation matrix.
    /// @param The new transformation matrix.
    void SetMatrix(const mat3x4& matrix) {
      m_matrix = matrix;
      m_inv_matrix = matrix.Inverse();
    }

    /// Get the transformation matrix.
    const mat3x4 Matrix() const {
      return m_matrix;
    }

    /// Get the inverse transformation matrix.
    const mat3x4 InvMatrix() const {
      return m_inv_matrix;
    }

    /// Set the material for this object.
    /// @param material The material to use (or NULL).
    void SetMaterial(mageray::Material* material) {
      m_material = material;
    }

    /// Set the material for this object.
    /// @param material The material to use (or NULL).
    const mageray::Material* Material() const {
      return m_material;
    }

    /// Find intersection between object and ray.
    /// @param ray The ray to shoot against the object (in world space).
    /// @param[in,out] hit Current closest hit information.
    /// @returns True if the ray intersects with the object.
    bool Intersect(const Ray& ray, HitInfo& hit) const;

    /// Fill out remaining hit information.
    /// @param ray The ray (in world space) used to intersect the object.
    /// @param[in,out] hit The HitInfo to fill out.
    void CompleteHitInfo(const Ray& ray, HitInfo& hit) const;

    /// Get the transformed bounding box for this object.
    /// @returns The bounding box (in world space) for this object.
    AABB TransformedBoundingBox() const;

  private:
    mat3x4 m_matrix;
    mat3x4 m_inv_matrix;

    mageray::Material* m_material;

    const Mesh* m_mesh;

    FORBID_COPY(Object);
};

} // namespace mageray

#endif // MAGERAY_OBJECT_H_
