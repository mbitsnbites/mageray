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

#include "aabb.h"

#include "base/platform.h"
#include "ray.h"

namespace mageray {

bool AABB::Intersect(const Ray& ray, scalar& closest_t) const {
  // Calculate the intersections with the three closest sides of the bounding
  // box.
  vec3 aabb_closest(Bound(ray.CloseMinMax()[vec3::X]).x,
                    Bound(ray.CloseMinMax()[vec3::Y]).y,
                    Bound(ray.CloseMinMax()[vec3::Z]).z);
  scalar t_x = (aabb_closest.x - ray.Origin().x) * ray.InvDirection().x;
  scalar t_y = (aabb_closest.y - ray.Origin().y) * ray.InvDirection().y;
  scalar t_z = (aabb_closest.z - ray.Origin().z) * ray.InvDirection().z;

  // The farthest hit is our candidate.
  vec3::Axis axis;
  scalar t;
  if (t_x > t_y) {
    if (t_x > t_z) {
      axis = vec3::X;
      t = t_x;
    } else {
      axis = vec3::Z;
      t = t_z;
    }
  } else {
    if (t_y > t_z) {
      axis = vec3::Y;
      t = t_y;
    } else {
      axis = vec3::Z;
      t = t_z;
    }
  }

  // Is this even a candidate? This is unlikely in a tree traversal since we
  // direct the traversal to try closest bounding boxes first.
  if (UNLIKELY(t > closest_t)) {
    return false;
  }

  // Calculate intersection point, and check if we're inside the bounds of the
  // side.
  vec3::Axis axis2 = vec3::NextAxis(axis);
  vec3::Axis axis3 = vec3::NextAxis(axis2);
  scalar a = ray.Origin()[axis2] + t * ray.Direction()[axis2];
  if (a < m_min[axis2] || a > m_max[axis2]) {
    return false;
  }
  scalar b = ray.Origin()[axis3] + t * ray.Direction()[axis3];
  if (b < m_min[axis3] || b > m_max[axis3]) {
    return false;
  }

  // If the intersection point was behind the ray origin, check that the
  // bounding box is not completely behind the ray origin.
  if (UNLIKELY(t < 0)) {
    vec3 aabb_far(Bound(ray.FarMinMax()[vec3::X]).x,
                  Bound(ray.FarMinMax()[vec3::Y]).y,
                  Bound(ray.FarMinMax()[vec3::Z]).z);
    if (ray.PointBehind(aabb_far)) {
      return false;
    }
  }

  // We had an intersection!
  closest_t = t;
  return true;
}

} // namespace mageray
