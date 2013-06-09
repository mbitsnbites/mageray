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
#include <thread>

#include "aabb.h"
#include "base/perf.h"
#include "base/platform.h"
#include "base/threads.h"

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
    const std::vector<Photon>::iterator stop, const Photon* parent,
    int threaded_depth) {
  // Empty recursion?
  if (start == stop) {
    return;
  }

  // Calculate bounding box for this branch.
  AABB aabb(start->position, start->position);
  for (auto it = start + 1; it != stop; it++) {
    aabb += it->position;
  }

  // Select the optimal axis to sort along.
  vec3::Axis axis = aabb.LargestAxis();

  if (!parent || parent->axis != axis) {
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
  }

  // Determine mid element (the point for this node in the tree).
  auto node = start + ((stop - start) / 2);
  node->SetAxis(axis);

  // Recursively build the two child trees.
  if (UNLIKELY(threaded_depth >= 1)) {
    // Spawn new threads when we're at the correct depth to better utilize
    // multi-core CPUs.
    std::thread t1 = std::thread(BuildSubTree, start, node, &*node,
        threaded_depth - 1);
    std::thread t2 = std::thread(BuildSubTree, node + 1, stop, &*node,
        threaded_depth - 1);
    t1.join();
    t2.join();
  } else {
    BuildSubTree(start, node, &*node, threaded_depth - 1);
    BuildSubTree(node + 1, stop, &*node, threaded_depth - 1);
  }

}

vec3 RecGetLightInRange(const vec3& position, const vec3& normal,
    const scalar range2, const AABB& aabb,
    const std::vector<Photon>::const_iterator start,
    const std::vector<Photon>::const_iterator stop) {
  vec3 color(0);

  // Empty recursion?
  if (start == stop) {
    return color;
  }

  // Determine mid element (the point for this node in the tree).
  auto node = start + ((stop - start) / 2);

  // Is this a point to collect?
  if (aabb.PointInside(node->position)) {
    scalar cos_alpha = -normal.Dot(node->direction);
    if (cos_alpha >= scalar(0.0)) {
      scalar r2 = (position - node->position).AbsSqr();
      if (r2 < range2) {
        color += node->color * (cos_alpha * (scalar(1.0) - r2 / range2));
      }
    }
  }

  // Are there any more child nodes?
  if (stop - start > 1) {
    // Recurse further?
    vec3::Axis axis = node->Axis();
    scalar p = node->position[axis];
    if (p >= aabb.Min()[axis]) {
      color += RecGetLightInRange(position, normal, range2, aabb, start, node);
    }
    if (p <= aabb.Max()[axis]) {
      color += RecGetLightInRange(position, normal, range2, aabb, node + 1,
          stop);
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
  DLOG("Number of photons in map: %d", m_size);

  // Select at which tree depth to split execution into parallell threads.
  // threaded_depth interpretation:
  //   0 -> Single threaded
  //   1 -> 2 threads
  //   2 -> 4 threads
  //   3 -> 8 threads
  //   etc.
  int threaded_depth;
  int concurrency = Thread::hardware_concurrency();
  if (concurrency > 8)
    threaded_depth = 4;
  else if (concurrency > 4)
    threaded_depth = 3;
  else if (concurrency > 2)
    threaded_depth = 2;
  else if (concurrency > 1)
    threaded_depth = 1;
  else
    threaded_depth = 0;
  DLOG("Hardware concurrency=%d, threaded_depth=%d", concurrency, threaded_depth);

  // Recursively build the KD tree (in place).
  BuildSubTree(m_photons.begin(), m_photons.begin() + m_size, NULL,
      threaded_depth);

  _perf.Done();

  // Determine the median distance between photons in the KD tree.
  DetermineMedianDistance();
}

vec3 PhotonMap::GetTotalLightInRange(const vec3& position,
    const vec3& normal, const scalar range) const {
  // Set up bounding box.
  AABB aabb(position - vec3(range), position + vec3(range));

  // Get contributing light from the photon map.
  auto start = m_photons.begin();
  return RecGetLightInRange(position, normal, range * range, aabb, start,
    start + m_size);
}

void PhotonMap::DetermineMedianDistance() {
  ScopedPerf _perf("Determine median distance");
  if (m_size < 2) {
    m_median_distance = scalar(0.0);
    return;
  }

  std::vector<scalar> distances(m_size);

  // TODO(mage): This is a very simple approximation. Should use a nearest
  // neighbour search instead.
  distances[0] = (m_photons[0].position -
      m_photons[m_size - 1].position).AbsSqr();
  for (unsigned i = 1; i < m_size; ++i) {
    distances[i] = (m_photons[i].position -
        m_photons[i - 1].position).AbsSqr();
  }

  // Calculate median distance.
  std::sort(distances.begin(), distances.end());
  m_median_distance = std::sqrt(distances[m_size / 2]);

  _perf.Done();

  DLOG("Median distance = %f", double(m_median_distance));
}

} // namespace mageray
