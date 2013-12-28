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

#ifndef MAGERAY_PHOTON_MAP_H_
#define MAGERAY_PHOTON_MAP_H_

#include <atomic>
#include <vector>

#include "base/log.h"
#include "base/types.h"
#include "aabb.h"
#include "vec.h"

namespace mageray {

struct Photon {
  vec3 position;
  vec3 direction;
  vec3 color;
  unsigned axis; // Really vec3::Axis, but we want to guarantee alignment.

  vec3::Axis Axis() const {
    return static_cast<vec3::Axis>(axis);
  }

  void SetAxis(const vec3::Axis _axis) {
    axis = static_cast<int>(_axis);
  }

};

class PhotonMap {
  public:
    PhotonMap() : m_capacity(0), m_count(0), m_size(0) {}

    /// Set the capacity of the photon map.
    /// @param capacity The number of photons that should fit in the map.
    /// @note This method must be called before using the map, and it can only
    /// be called once.
    void SetCapacity(const int capacity) {
      ASSERT(m_capacity == 0, "Can't change the capacity of a PhotonMap.");
      m_capacity = capacity;
      m_photons.resize(capacity);
    }

    /// @returns The photon map capacity.
    int Capacity() const {
      return m_capacity;
    }

    /// Get the next un-initialized photon.
    /// @returns a pointer to an un-initialized photon if a new photon was
    /// returned, or NULL if the photon map was full.
    /// @note This method is thread safe.
    Photon* NextPhoton() {
      int index = m_count++;
      return index < m_capacity ? &m_photons[index] : NULL;
    }

    /// Check if the photon map is full.
    /// @returns true if the photon map was full.
    /// @note This method is thread safe.
    bool IsFull() {
      return m_count >= m_capacity;
    }

    /// Build a KD tree of the collected photons.
    void BuildKDTree();

    /// @returns true if the photon map contains a KD tree.
    bool HasPhotons() const {
      return m_size > 0;
    }

    /// @returns the median distance between photons.
    scalar MedianDistance() const {
      return m_median_distance;
    }

  private:
    struct CollectedPhoton {
      bool operator<(const CollectedPhoton& other) const {
        return distance < other.distance;
      }

      const Photon* photon;
      scalar distance;
    };

  public:
    class Collector {
      public:
        Collector(const PhotonMap& map, unsigned max_count, scalar max_range) :
            m_photon_map(&map), m_collected(max_count), m_max_range(max_range) {}

        /// Get the total light at the given surface point.
        /// @param position The position in space to query.
        /// @param normal The surface normal of the surface to query.
        /// @returns The total color (photon energy).
        vec3 CollectLight(const vec3& position, const vec3& normal);

      private:
        const PhotonMap* m_photon_map;
        std::vector<CollectedPhoton> m_collected;
        scalar m_max_range;
    };

  private:
    static void RecCollectPhotons(std::vector<CollectedPhoton>& collected,
                                  unsigned& count, const vec3& position,
                                  const vec3& normal, scalar& range2,
                                  AABB& aabb,
                                  const std::vector<Photon>::const_iterator start,
                                  const std::vector<Photon>::const_iterator stop);

    unsigned CollectPhotons(std::vector<CollectedPhoton>& collected,
                            const scalar range, const vec3& position,
                            const vec3& normal) const;

    void DetermineMedianDistance();

    /// The number of elements that fit in the photon vector.
    int m_capacity;

    /// The current photon count (may exceed the capacity).
    std::atomic_int m_count;

    /// The actual number of photons (defined by BuildKDTree).
    unsigned m_size;

    /// The median distance between photons in the tree (defined by
    /// DetermineMedianDistance).
    scalar m_median_distance;

    std::vector<Photon> m_photons;
};

} // namespace mageray

#endif // MAGERAY_PHOTON_MAP_H_
