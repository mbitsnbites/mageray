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

#include "ray.h"

bool AABB::Intersect(const Ray& ray, scalar& closest_t) const {
  // Calculate the intersections with the three closest sides of the bounding
  // box.
  vec3 aabb_closest(m_bounds[ray.CloseSides()[X]],
                    m_bounds[ray.CloseSides()[Y]],
                    m_bounds[ray.CloseSides()[Z]]);
  scalar t_x = (aabb_closest.x - ray.Origin().x) * ray.InvDirection().x;
  scalar t_y = (aabb_closest.y - ray.Origin().y) * ray.InvDirection().y;
  scalar t_z = (aabb_closest.z - ray.Origin().z) * ray.InvDirection().z;

  // The farthest hit is our candidate.
  Axis axis;
  scalar t;
  if (t_x > t_y) {
    if (t_x > t_z) {
      axis = X;
      t = t_x;
    } else {
      axis = Z;
      t = t_z;
    }
  } else {
    if (t_y > t_z) {
      axis = Y;
      t = t_y;
    } else {
      axis = Z;
      t = t_z;
    }
  }

  // Is this even a candidate?
  if (t > closest_t) {
    return false;
  }

  // Calculate intersection point, and check if we're inside the bounds of the
  // side.
  switch (axis) {
    case X: {
      scalar y = ray.Origin().y + t * ray.Direction().y;
      if (y < m_bounds[YMIN] || y > m_bounds[YMAX]) {
        return false;
      }
      scalar z = ray.Origin().z + t * ray.Direction().z;
      if (z < m_bounds[ZMIN] || z > m_bounds[ZMAX]) {
        return false;
      }
      break;
    }
    case Y: {
      scalar x = ray.Origin().x + t * ray.Direction().x;
      if (x < m_bounds[XMIN] || x > m_bounds[XMAX]) {
        return false;
      }
      scalar z = ray.Origin().z + t * ray.Direction().z;
      if (z < m_bounds[ZMIN] || z > m_bounds[ZMAX]) {
        return false;
      }
      break;
    }
    case Z: {
      scalar x = ray.Origin().x + t * ray.Direction().x;
      if (x < m_bounds[XMIN] || x > m_bounds[XMAX]) {
        return false;
      }
      scalar y = ray.Origin().y + t * ray.Direction().y;
      if (y < m_bounds[YMIN] || y > m_bounds[YMAX]) {
        return false;
      }
      break;
    }
  }

  // If the intersection point was behind the ray origin, check that the
  // bounding box is not completely behind the ray origin.
  if (t < 0) {
    vec3 aabb_far(m_bounds[ray.FarSides()[X]],
                  m_bounds[ray.FarSides()[Y]],
                  m_bounds[ray.FarSides()[Z]]);
    if (ray.PointBehind(aabb_far)) {
      return false;
    }
  }

  // We had an intersection!
  closest_t = t;
  return true;
}
