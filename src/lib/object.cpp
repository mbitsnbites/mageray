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

#include "object.h"

#include <cmath>

#include "base/platform.h"

namespace mageray {

Object::Object(const Mesh* mesh) : m_material(NULL), m_mesh(mesh) {
  ASSERT(m_mesh, "Undefined mesh.");
  m_matrix = mat3x4::Identity();
  m_inv_matrix = mat3x4::Identity();
}

void Object::Translate(const vec3& t) {
  m_matrix = m_matrix * mat3x4::Translate(t);
  m_inv_matrix = m_matrix.Inverse();
}

void Object::Scale(const vec3& s) {
  m_matrix = m_matrix * mat3x4::Scale(s);
  m_inv_matrix = m_matrix.Inverse();
}

void Object::Rotate(const vec3& r) {
  m_matrix = m_matrix * mat3x4::Rotate(r);
  m_inv_matrix = m_matrix.Inverse();
}

bool Object::Intersect(const Ray& ray, HitInfo& hit) const {
  // Transform ray into object space.
  const vec3 origin = m_inv_matrix.TransformPoint(ray.Origin());
  const vec3 direction = m_inv_matrix.TransformDirection(ray.Direction());
  const Ray transformed_ray(origin, direction);

  // Perform intersection.
  bool got_hit = m_mesh->Intersect(transformed_ray, hit);
  if (got_hit) {
    hit.object = this;
  }

  return got_hit;
}

void Object::CompleteHitInfo(const Ray& ray, HitInfo& hit) const {
  // Calculate intersection point (in world coordinates).
  hit.point = ray.Origin() + ray.Direction() * hit.t;

  // Get hit information in object space.
  m_mesh->CompleteHitInfo(hit);

  // Transform normal to world space.
  hit.normal = m_matrix.TransformDirection(hit.normal).Normalize();
}

AABB Object::TransformedBoundingBox() const {
  // Get the object space bounding box.
  const AABB original = m_mesh->BoundingBox();

  // Construct the 8 transformed corner points of the bounding box.
  const vec3 corners[8] = {
    m_matrix * vec3(original.Min().x, original.Min().y, original.Min().z),
    m_matrix * vec3(original.Min().x, original.Min().y, original.Max().z),
    m_matrix * vec3(original.Min().x, original.Max().y, original.Min().z),
    m_matrix * vec3(original.Min().x, original.Max().y, original.Max().z),
    m_matrix * vec3(original.Max().x, original.Min().y, original.Min().z),
    m_matrix * vec3(original.Max().x, original.Min().y, original.Max().z),
    m_matrix * vec3(original.Max().x, original.Max().y, original.Min().z),
    m_matrix * vec3(original.Max().x, original.Max().y, original.Max().z)
  };

  // Find the bounding box that includes all the 8 transformed points.
  vec3 min(corners[0]), max(corners[0]);
  for (int i = 1; i < 8; ++i) {
    if (corners[i].x < min.x) min.x = corners[i].x;
    if (corners[i].x > max.x) max.x = corners[i].x;
    if (corners[i].y < min.y) min.y = corners[i].y;
    if (corners[i].y > max.y) max.y = corners[i].y;
    if (corners[i].z < min.z) min.z = corners[i].z;
    if (corners[i].z > max.z) max.z = corners[i].z;
  }

  return AABB(min, max);
}

} // namespace mageray
