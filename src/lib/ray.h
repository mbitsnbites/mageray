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

#ifndef MAGERAY_RAY_H_
#define MAGERAY_RAY_H_

#include <ostream>

#include "aabb.h"
#include "vec.h"

namespace mageray {

/// Axis aligned bounding box.
class Ray {
  public:
    Ray() {}

    Ray(const vec3& origin, const vec3& direction) :
        m_origin(origin),
        m_direction(direction) {
      // Calculate inverse of the ray direction.
      m_inv_direction = vec3(scalar(1.0) / direction.x,
                             scalar(1.0) / direction.y,
                             scalar(1.0) / direction.z);

      // Determine closest and farthest AABB sides for the given direction.
      if (direction.x >= 0) {
        m_close_min_max[vec3::X] = AABB::MIN;
        m_far_min_max[vec3::X] = AABB::MAX;
      } else {
        m_close_min_max[vec3::X] = AABB::MAX;
        m_far_min_max[vec3::X] = AABB::MIN;
      }
      if (direction.y >= 0) {
        m_close_min_max[vec3::Y] = AABB::MIN;
        m_far_min_max[vec3::Y] = AABB::MAX;
      } else {
        m_close_min_max[vec3::Y] = AABB::MAX;
        m_far_min_max[vec3::Y] = AABB::MIN;
      }
      if (direction.z >= 0) {
        m_close_min_max[vec3::Z] = AABB::MIN;
        m_far_min_max[vec3::Z] = AABB::MAX;
      } else {
        m_close_min_max[vec3::Z] = AABB::MAX;
        m_far_min_max[vec3::Z] = AABB::MIN;
      }

      // Calculate the plane equation parameter for the plane that has the ray
      // direction as normal and the ray origin as one of its points.
      m_plane_d = origin.Dot(direction);
    }

    /// @returns The ray origin.
    const vec3& Origin() const {
      return m_origin;
    }

    /// @returns The ray direction.
    const vec3& Direction() const {
      return m_direction;
    }

    /// @returns The inverse ray direction.
    const vec3& InvDirection() const {
      return m_inv_direction;
    }

    /// Check if a point is behind the ray origin.
    /// @returns True if the given point is behind the plane described by the
    /// ray origin and the ray direction.
    bool PointBehind(const vec3& point) const {
      return m_direction.Dot(point) < m_plane_d;
    }

    /// @returns A three element array (x, y, z) holding the closest sides of
    /// an AABB for this ray's direction.
    const AABB::MinMax* CloseMinMax() const {
      return m_close_min_max;
    }

    /// @returns A three element array (x, y, z) holding the farthest sides of
    /// an AABB for this ray's direction.
    const AABB::MinMax* FarMinMax() const {
      return m_far_min_max;
    }

    friend std::ostream& operator<<(std::ostream& os, const Ray& ray) {
      os << ray.m_origin << "," << ray.m_direction;
      return os;
    }

  private:
    vec3 m_origin;
    vec3 m_direction;
    vec3 m_inv_direction;
    scalar m_plane_d;
    AABB::MinMax m_close_min_max[3];
    AABB::MinMax m_far_min_max[3];
};

} // namespace mageray

#endif // MAGERAY_RAY_H_
