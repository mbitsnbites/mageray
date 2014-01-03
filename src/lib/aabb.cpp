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
  const vec3 aabb_closest(Bound(ray.CloseMinMax()[vec3::X]).x,
                          Bound(ray.CloseMinMax()[vec3::Y]).y,
                          Bound(ray.CloseMinMax()[vec3::Z]).z);

  const vec3 t3 = (aabb_closest - ray.Origin()) * ray.InvDirection();

  // The farthest hit is our candidate.
  vec3::Axis axis;
  scalar t;
  if (t3.x > t3.y) {
    if (t3.x > t3.z) {
      axis = vec3::X;
      t = t3.x;
    } else {
      axis = vec3::Z;
      t = t3.z;
    }
  } else {
    if (t3.y > t3.z) {
      axis = vec3::Y;
      t = t3.y;
    } else {
      axis = vec3::Z;
      t = t3.z;
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
  if (a < Min()[axis2] || a > Max()[axis2]) {
    return false;
  }
  scalar b = ray.Origin()[axis3] + t * ray.Direction()[axis3];
  if (b < Min()[axis3] || b > Max()[axis3]) {
    return false;
  }

  // If the intersection point was behind the ray origin, check that the
  // ray origin is inside the bounding box (otherwise the bounding box is
  // behind the ray).
  // TODO(m): If we do inside AABB tests before intersection tests, this check
  // should be unnecessary.
  if (UNLIKELY(t < 0)) {
    if (!PointInside(ray.Origin())) {
      return false;
    }
  }

  // We had an intersection!
  closest_t = t;
  return true;
}

} // namespace mageray
