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
    AABB() {}

    AABB(const scalar minx, const scalar miny, const scalar minz,
         const scalar maxx, const scalar maxy, const scalar maxz) {
      m_bounds[XMIN] = minx;
      m_bounds[YMIN] = miny;
      m_bounds[ZMIN] = minz;
      m_bounds[XMAX] = maxx;
      m_bounds[YMAX] = maxy;
      m_bounds[ZMAX] = maxz;
    }

    AABB(const vec3& min, const vec3& max) {
      m_bounds[XMIN] = min.x;
      m_bounds[YMIN] = min.y;
      m_bounds[ZMIN] = min.z;
      m_bounds[XMAX] = max.x;
      m_bounds[YMAX] = max.y;
      m_bounds[ZMAX] = max.z;
    }

    enum Bound {
      XMIN = 0,
      YMIN = 1,
      ZMIN = 2,
      XMAX = 3,
      YMAX = 4,
      ZMAX = 5
    };

    enum Axis {
      X = 0,
      Y = 1,
      Z = 2
    };

    static Bound MinBound(const Axis axis) {
      return static_cast<Bound>(axis);
    }

    static Bound MaxBound(const Axis axis) {
      return static_cast<Bound>(axis + 3);
    }

    /// Minimum coordinate for this bounding box.
    vec3 Min() const {
      return vec3(m_bounds[XMIN], m_bounds[YMIN], m_bounds[ZMIN]);
    }

    /// Maxium coordinate for this bounding box.
    vec3 Max() const {
      return vec3(m_bounds[XMAX], m_bounds[YMAX], m_bounds[ZMAX]);
    }

    /// Maxium side axis.
    Axis LargestAxis() const {
      scalar dx = m_bounds[XMAX] - m_bounds[XMIN];
      scalar dy = m_bounds[YMAX] - m_bounds[YMIN];
      scalar dz = m_bounds[ZMAX] - m_bounds[ZMIN];
      if (dx >= dy) {
        return dx >= dz ? X : Z;
      } else {
        return dy >= dz ? Y : Z;
      }
    }

    /// Union of two bounding boxes.
    AABB& operator+=(const AABB& other) {
      if (other[XMIN] < m_bounds[XMIN]) m_bounds[XMIN] = other[XMIN];
      if (other[YMIN] < m_bounds[YMIN]) m_bounds[YMIN] = other[YMIN];
      if (other[ZMIN] < m_bounds[ZMIN]) m_bounds[ZMIN] = other[ZMIN];
      if (other[XMAX] > m_bounds[XMAX]) m_bounds[XMAX] = other[XMAX];
      if (other[YMAX] > m_bounds[YMAX]) m_bounds[YMAX] = other[YMAX];
      if (other[ZMAX] > m_bounds[ZMAX]) m_bounds[ZMAX] = other[ZMAX];
      return *this;
    }

    scalar& operator[](Bound bound) {
      return m_bounds[bound];
    }

    const scalar& operator[](Bound bound) const {
      return m_bounds[bound];
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
      return point.x >= m_bounds[XMIN] && point.x <= m_bounds[XMAX] &&
          point.y >= m_bounds[YMIN] && point.y <= m_bounds[YMAX] &&
          point.z >= m_bounds[ZMIN] && point.z <= m_bounds[ZMAX];
    }

    friend std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
      os << aabb.Min() << "->" << aabb.Max();
      return os;
    }

  private:
    scalar m_bounds[6];
};

} // namespace mageray

#endif // MAGERAY_AABB_H_
