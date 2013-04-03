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

#include "photon_map.h"

#include <algorithm>

#include "aabb.h"

namespace mageray {

namespace {

bool ComparePhotonX(const Photon& p1, const Photon& p2) {
  return p1.position.x < p2.position.x;
}

bool ComparePhotonY(const Photon& p1, const Photon& p2) {
  return p1.position.y < p2.position.y;
}

bool ComparePhotonZ(const Photon& p1, const Photon& p2) {
  return p1.position.z < p2.position.z;
}

void BuildSubTree(const std::vector<Photon>::iterator start,
    const std::vector<Photon>::iterator stop, Photon::Axis axis) {
  // Empty recursion?
  if (start == stop) {
    return;
  }

  // Sort all elements according to the selected axis.
  switch(axis) {
    case Photon::X:
      std::sort(start, stop, ComparePhotonX);
      break;
    case Photon::Y:
      std::sort(start, stop, ComparePhotonY);
      break;
    case Photon::Z:
      std::sort(start, stop, ComparePhotonZ);
      break;
  }

  // Determine mid element (the point for this node in the tree).
  auto mid = start + ((stop - start) / 2);

  // Select the next axis to sort along.
  // TODO(mage): We might want to select the sort axis based on the sub tree
  // bounding box (or similar) instead. That would require an additional field
  // in the Photon struct so that the tree lookup knows how to traverse the
  // tree.
  Photon::Axis next_axis = static_cast<Photon::Axis>((axis + 1) % 3);

  // Recursively build the two child trees.
  BuildSubTree(start, mid, next_axis);
  BuildSubTree(mid + 1, stop, next_axis);
}

bool inline PhotonInRange(const Photon& photon, const vec3& position,
    const vec3& normal, const scalar range2, const scalar plane_d) {
  // Check if the point is in the plane (roughly), and within the circle.
  return std::abs(photon.position.Dot(normal) - plane_d) <= EPSILON &&
      (position - photon.position).AbsSqr() <= range2;
}

int RecGetLightInRange(vec3& color, vec3& direction,
    const vec3& position, const vec3& normal, const scalar range2,
    const scalar plane_d, const AABB& aabb,
    const std::vector<Photon>::iterator start,
    const std::vector<Photon>::iterator stop, Photon::Axis axis) {
  // Empty recursion?
  if (start == stop) {
    return 0;
  }

  // Determine mid element (the point for this node in the tree).
  auto mid = start + ((stop - start) / 2);

  // Is this a point to collect?
  int count = 0;
  if (aabb.PointInside(mid->position)) {
    if (PhotonInRange(*mid, position, normal, range2, plane_d)) {
      color += mid->color;
      direction += mid->direction;
      count = 1;
    }
  }

  // Are there any more child nodes?
  if (stop - start > 1) {
    // Select the next split axis.
    Photon::Axis next_axis = static_cast<Photon::Axis>((axis + 1) % 3);

    // Recurse further?
    switch(axis) {
      case Photon::X:
        if (mid->position.x >= aabb[AABB::XMIN]) {
          count += RecGetLightInRange(color, direction, position, normal,
              range2, plane_d, aabb, start, mid, next_axis);
        }
        if (mid->position.x <= aabb[AABB::XMAX]) {
          count += RecGetLightInRange(color, direction, position, normal,
              range2, plane_d, aabb, mid, stop, next_axis);
        }
        break;
      case Photon::Y:
        if (mid->position.y >= aabb[AABB::YMIN]) {
          count += RecGetLightInRange(color, direction, position, normal,
              range2, plane_d, aabb, start, mid, next_axis);
        }
        if (mid->position.y <= aabb[AABB::YMAX]) {
          count += RecGetLightInRange(color, direction, position, normal,
              range2, plane_d, aabb, mid, stop, next_axis);
        }
        break;
      case Photon::Z:
        if (mid->position.z >= aabb[AABB::ZMIN]) {
          count += RecGetLightInRange(color, direction, position, normal,
              range2, plane_d, aabb, start, mid, next_axis);
        }
        if (mid->position.z <= aabb[AABB::ZMAX]) {
          count += RecGetLightInRange(color, direction, position, normal,
              range2, plane_d, aabb, mid, stop, next_axis);
        }
        break;
    }
  }

  return count;
}

} // anonymous namespace


void PhotonMap::BuildKDTree() {
  // Get the actual number of photons in the photon array.
  int count = m_count;
  m_size = std::min(m_capacity, count);

  // Recursively build the KD tree (in place).
  BuildSubTree(m_photons.begin(), m_photons.begin() + m_size, Photon::X);
}

int PhotonMap::GetTotalLightInRange(vec3& color, vec3& direction,
    const vec3& position, const vec3& normal, const scalar range) {
  // Set up bounding box.
  AABB aabb(position - vec3(range), position + vec3(range));

  // Construct plane equation from position and normal.
  scalar plane_d = position.Dot(normal);

  // Get contributing light from the photon map.
  vec3 color_sum(0), direction_sum(0);
  auto start = m_photons.begin();
  auto stop = start + m_size;
  int count = RecGetLightInRange(color_sum, direction_sum, position, normal,
      range * range, plane_d, aabb, start, stop, Photon::X);

  // Return output parameters.
  color = color_sum;
  direction = direction_sum.Normalize();

  return count;
}

} // namespace mageray
