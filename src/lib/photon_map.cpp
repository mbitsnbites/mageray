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
#include "base/perf.h"

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
    const std::vector<Photon>::iterator stop, vec3::Axis axis) {
  // Empty recursion?
  if (start == stop) {
    return;
  }

  // Sort all elements according to the selected axis.
  switch(axis) {
    case vec3::X:
      std::sort(start, stop, ComparePhotonX);
      break;
    case vec3::Y:
      std::sort(start, stop, ComparePhotonY);
      break;
    case vec3::Z:
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
  vec3::Axis next_axis = vec3::NextAxis(axis);

  // Recursively build the two child trees.
  BuildSubTree(start, mid, next_axis);
  BuildSubTree(mid + 1, stop, next_axis);
}

vec3 RecGetLightInRange(const vec3& position, const vec3& normal,
    const scalar range2, const AABB& aabb,
    const std::vector<Photon>::const_iterator start,
    const std::vector<Photon>::const_iterator stop, const vec3::Axis axis) {
  vec3 color(0);

  // Empty recursion?
  if (start == stop) {
    return color;
  }

  // Determine mid element (the point for this node in the tree).
  auto mid = start + ((stop - start) / 2);

  // Is this a point to collect?
  if (aabb.PointInside(mid->position)) {
    scalar cos_alpha = -normal.Dot(mid->direction);
    if (cos_alpha >= scalar(0.0)) {
      scalar r2 = (position - mid->position).AbsSqr();
      if (r2 < range2) {
        color += mid->color * (cos_alpha * (scalar(1.0) - r2 / range2));
      }
    }
  }

  // Are there any more child nodes?
  if (stop - start > 1) {
    // Select the next split axis.
    vec3::Axis next_axis = vec3::NextAxis(axis);

    // Recurse further?
    if (mid->position[axis] >= aabb.Min()[axis]) {
      color += RecGetLightInRange(position, normal,
          range2, aabb, start, mid, next_axis);
    }
    if (mid->position[axis] <= aabb.Max()[axis]) {
      color += RecGetLightInRange(position, normal,
          range2, aabb, mid + 1, stop, next_axis);
    }
  }

  return color;
}

} // anonymous namespace


void PhotonMap::BuildKDTree() {
  ScopedPerf _perf("BuildKDTree");

  // Get the actual number of photons in the photon array.
  int count = m_count;
  m_size = std::min(m_capacity, count);

  // Recursively build the KD tree (in place).
  BuildSubTree(m_photons.begin(), m_photons.begin() + m_size, vec3::X);

  _perf.Done();
}

vec3 PhotonMap::GetTotalLightInRange(const vec3& position,
    const vec3& normal, const scalar range) const {
  // Set up bounding box.
  AABB aabb(position - vec3(range), position + vec3(range));

  // Get contributing light from the photon map.
  auto start = m_photons.begin();
  auto stop = start + m_size;
  return RecGetLightInRange(position, normal, range * range, aabb, start, stop,
      vec3::X);
}

} // namespace mageray
