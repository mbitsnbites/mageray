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

#include "base/types.h"
#include "vec.h"

namespace mageray {

struct Photon {
  vec3 position;
  vec3 direction;
  vec3 color;
};

class PhotonMap {
  public:
    PhotonMap(const int size);

    ~PhotonMap();

    int Size() const {
      return m_size;
    }

    int Count() const {
      return m_count;
    }

    /// Get the next un-initialized photon.
    /// @returns a pointer to an un-initialized photon if a new photon was
    /// returned, or NULL if the photon map was full.
    /// @note This method is thread safe.
    Photon* NextPhoton() {
      int index = m_count++;
      return index < m_size ? &m_photons[index] : NULL;
    }

    /// Build a KD tree of the collected photons.
    void BuildKDTree();

    /// Get the total light in the given range.
    /// @param[out] color The total color (photon energy).
    /// @param[out] direction The average light direction.
    /// @param position The position in space to query.
    /// @param normal The surface normal of the surface to query.
    /// @param range The radius to query.
    /// @returns The number of photons collected.
    int GetTotalLightInRange(vec3& color, vec3& direction, const vec3& position,
        const vec3& normal, const scalar range) const;

  private:
    const int m_size;
    std::atomic_int m_count;

    Photon* m_photons;
};

} // namespace mageray

#endif // MAGERAY_PHOTON_MAP_H_
