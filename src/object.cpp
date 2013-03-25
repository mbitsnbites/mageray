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

Object::Object() : m_material(NULL) {
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
  bool got_hit = IntersectInObjectSpace(transformed_ray, hit);
  if (got_hit) {
    hit.object = this;
  }

  return got_hit;
}

void Object::CompleteHitInfo(const Ray& ray, HitInfo& hit) const {
  // Calculate intersection point (in world coordinates).
  hit.point = ray.Origin() + ray.Direction() * hit.t;

  // Get hit information in object space.
  CompleteHitInfoInObjectSpace(hit);

  // Transform normal to world space.
  hit.normal = m_matrix.TransformDirection(hit.normal);
}

void Object::GetBoundingBox(AABB& aabb) const {
  // Get the object space bounding box.
  AABB original;
  GetBoundingBoxInObjectSpace(original);

  // Construct the 8 corner points of the bounding box.
  vec3 corners[8];
  corners[0] = vec3(original[AABB::XMIN], original[AABB::YMIN], original[AABB::ZMIN]);
  corners[1] = vec3(original[AABB::XMIN], original[AABB::YMIN], original[AABB::ZMAX]);
  corners[2] = vec3(original[AABB::XMIN], original[AABB::YMAX], original[AABB::ZMIN]);
  corners[3] = vec3(original[AABB::XMIN], original[AABB::YMAX], original[AABB::ZMAX]);
  corners[4] = vec3(original[AABB::XMAX], original[AABB::YMIN], original[AABB::ZMIN]);
  corners[5] = vec3(original[AABB::XMAX], original[AABB::YMIN], original[AABB::ZMAX]);
  corners[6] = vec3(original[AABB::XMAX], original[AABB::YMAX], original[AABB::ZMIN]);
  corners[7] = vec3(original[AABB::XMAX], original[AABB::YMAX], original[AABB::ZMAX]);

  // Transform all the corners.
  for (int i = 0; i < 8; ++i) {
    corners[i] = m_matrix * corners[i];
  }

  // Find the bounding box that includes all the 8 transformed points.
  aabb[AABB::XMIN] = aabb[AABB::XMAX] = corners[0].x;
  aabb[AABB::YMIN] = aabb[AABB::YMAX] = corners[0].y;
  aabb[AABB::ZMIN] = aabb[AABB::ZMAX] = corners[0].z;
  for (int i = 1; i < 8; ++i) {
    if (corners[i].x < aabb[AABB::XMIN]) aabb[AABB::XMIN] = corners[i].x;
    if (corners[i].x > aabb[AABB::XMAX]) aabb[AABB::XMAX] = corners[i].x;
    if (corners[i].y < aabb[AABB::YMIN]) aabb[AABB::YMIN] = corners[i].y;
    if (corners[i].y > aabb[AABB::YMAX]) aabb[AABB::YMAX] = corners[i].y;
    if (corners[i].z < aabb[AABB::ZMIN]) aabb[AABB::ZMIN] = corners[i].z;
    if (corners[i].z > aabb[AABB::ZMAX]) aabb[AABB::ZMAX] = corners[i].z;
  }
}

bool MeshObject::IntersectInObjectSpace(const Ray& ray, HitInfo& hit) const {
  if (LIKELY(m_mesh)) {
    return m_mesh->Intersect(ray, hit);
  }
  return false;
}

void MeshObject::CompleteHitInfoInObjectSpace(HitInfo& hit) const {
  m_mesh->CompleteHitInfo(hit);
}

void MeshObject::GetBoundingBoxInObjectSpace(AABB& aabb) const {
  if (LIKELY(m_mesh)) {
    aabb = m_mesh->BoundingBox();
  } else {
    aabb[AABB::XMIN] = aabb[AABB::YMIN] = aabb[AABB::ZMIN] = 0.0;
    aabb[AABB::XMAX] = aabb[AABB::YMAX] = aabb[AABB::ZMAX] = 0.0;
  }
}

bool SphereObject::IntersectInObjectSpace(const Ray& ray, HitInfo& hit) const {
  scalar a = ray.Direction().Dot(ray.Direction());
  scalar b = -ray.Direction().Dot(ray.Origin());
  scalar c = ray.Origin().Dot(ray.Origin()) - m_radius_squared;

  scalar root_term = b * b - a * c;
  if (root_term < 0.0) {
    return false;
  }
  root_term = std::sqrt(root_term);

  scalar t = b > root_term ? b - root_term : b + root_term;
  t /= a;

  if (t < EPSILON || t >= hit.t) {
    return false;
  }

  // Store object space point for later (normal calculation).
  hit.object_space_point = ray.Origin() + ray.Direction() * t;

  // We had a hit.
  hit.t = t;
  return true;
}

void SphereObject::CompleteHitInfoInObjectSpace(HitInfo& hit) const {
  // The normal is the normalized point in space.
  hit.normal = hit.object_space_point * m_inv_radius;

  // The U/V coordinate is undefined.
  hit.uv = vec2(0);
}

void SphereObject::GetBoundingBoxInObjectSpace(AABB& aabb) const {
  scalar radius = std::sqrt(m_radius_squared);
  aabb[AABB::XMIN] = aabb[AABB::YMIN] = aabb[AABB::ZMIN] = -radius;
  aabb[AABB::XMAX] = aabb[AABB::YMAX] = aabb[AABB::ZMAX] = radius;
}
