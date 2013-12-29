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

  // Construct the 8 corner points of the bounding box.
  vec3 corners[8];
  corners[0] = vec3(original.Min().x, original.Min().y, original.Min().z);
  corners[1] = vec3(original.Min().x, original.Min().y, original.Max().z);
  corners[2] = vec3(original.Min().x, original.Max().y, original.Min().z);
  corners[3] = vec3(original.Min().x, original.Max().y, original.Max().z);
  corners[4] = vec3(original.Max().x, original.Min().y, original.Min().z);
  corners[5] = vec3(original.Max().x, original.Min().y, original.Max().z);
  corners[6] = vec3(original.Max().x, original.Max().y, original.Min().z);
  corners[7] = vec3(original.Max().x, original.Max().y, original.Max().z);

  // Transform all the corners.
  for (int i = 0; i < 8; ++i) {
    corners[i] = m_matrix * corners[i];
  }

  // Find the bounding box that includes all the 8 transformed points.
  AABB aabb;
  aabb.Min().x = aabb.Max().x = corners[0].x;
  aabb.Min().y = aabb.Max().y = corners[0].y;
  aabb.Min().z = aabb.Max().z = corners[0].z;
  for (int i = 1; i < 8; ++i) {
    if (corners[i].x < aabb.Min().x) aabb.Min().x = corners[i].x;
    if (corners[i].x > aabb.Max().x) aabb.Max().x = corners[i].x;
    if (corners[i].y < aabb.Min().y) aabb.Min().y = corners[i].y;
    if (corners[i].y > aabb.Max().y) aabb.Max().y = corners[i].y;
    if (corners[i].z < aabb.Min().z) aabb.Min().z = corners[i].z;
    if (corners[i].z > aabb.Max().z) aabb.Max().z = corners[i].z;
  }

  return aabb;
}

} // namespace mageray
