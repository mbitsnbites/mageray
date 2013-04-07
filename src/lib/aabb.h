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

#ifndef MAGERAY_AABB_H_
#define MAGERAY_AABB_H_

#include <ostream>

#include "base/types.h"
#include "vec.h"

namespace mageray {

class Ray;

/// Axis aligned bounding box.
class AABB {
  public:
    enum MinMax {
      MIN = 0,
      MAX = 1
    };

    AABB() {}

    AABB(const vec3& min, const vec3& max) : m_min(min), m_max(max) {}

    AABB(const scalar minx, const scalar miny, const scalar minz,
         const scalar maxx, const scalar maxy, const scalar maxz) {
      m_min.x = minx;
      m_min.y = miny;
      m_min.z = minz;
      m_max.x = maxx;
      m_max.y = maxy;
      m_max.z = maxz;
    }

    /// Minimum coordinate for this bounding box.
    vec3& Min() {
      return m_min;
    }

    /// Minimum coordinate for this bounding box.
    const vec3& Min() const {
      return m_min;
    }

    /// Maxium coordinate for this bounding box.
    vec3& Max() {
      return m_max;
    }

    /// Maxium coordinate for this bounding box.
    const vec3& Max() const {
      return m_max;
    }

    const vec3& Bound(const MinMax m) const {
      return m ? m_max : m_min;
    }

    /// Maxium side axis.
    vec3::Axis LargestAxis() const {
      vec3 d = m_max - m_min;
      if (d.x >= d.y) {
        return d.x >= d.z ? vec3::X : vec3::Z;
      } else {
        return d.y >= d.z ? vec3::Y : vec3::Z;
      }
    }

    /// Union of two bounding boxes.
    AABB& operator+=(const AABB& other) {
      if (other.m_min.x < m_min.x) m_min.x = other.m_min.x;
      if (other.m_min.y < m_min.y) m_min.y = other.m_min.y;
      if (other.m_min.z < m_min.z) m_min.z = other.m_min.z;
      if (other.m_max.x > m_max.x) m_max.x = other.m_max.x;
      if (other.m_max.y > m_max.y) m_max.y = other.m_max.y;
      if (other.m_max.z > m_max.z) m_max.z = other.m_max.z;
      return *this;
    }

    /// Check if a ray intersects this bounding box.
    /// @param ray The ray.
    /// @param[in,out] closest_t The closes intersection distance this far.
    /// If an intersection occurred, closest_t will be updated accordingly.
    bool Intersect(const Ray& ray, scalar& closest_t) const;

    /// Check if a point is inside this bounding box.
    /// @param point The point to check.
    /// @returns true if the point is inside the bounding box.
    bool PointInside(const vec3& point) const {
      return point.x >= m_min.x && point.x <= m_max.x &&
          point.y >= m_min.y && point.y <= m_max.y &&
          point.z >= m_min.z && point.z <= m_max.z;
    }

    friend std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
      os << aabb.m_min << "->" << aabb.m_max;
      return os;
    }

  private:
    vec3 m_min;
    vec3 m_max;
};

} // namespace mageray

#endif // MAGERAY_AABB_H_
