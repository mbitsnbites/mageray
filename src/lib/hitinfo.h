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

#ifndef MAGERAY_HITINFO_H_
#define MAGERAY_HITINFO_H_

#include "base/types.h"
#include "vec.h"

namespace mageray {

class Object;
class Triangle;

struct HitInfo {
  /// Create a new HitInfo object.
  /// The HitInfo object represents a "no hit" (i.e. infinitely distant).
  static HitInfo CreateNoHit() {
    HitInfo hit_info;
    hit_info.t = MAX_DISTANCE;
    return hit_info;
  }

  /// Create a new HitInfo object.
  /// The HitInfo object represents a valid intersecion interval between
  /// 0 and distance.
  static HitInfo CreateShadowTest(const scalar distance) {
    HitInfo hit_info;
    hit_info.t = distance;
    return hit_info;
  }

  const Object* object;     ///< The object that was hit.
  scalar t;                 ///< Closest intersection t so far.

  // Triangle mesh intersection information.
  const Triangle* triangle; ///< Which triangle was intersected.
  vec2 tri_uv;              ///< Triangle space U/V info (from intersection).

  vec3 point;               ///< Point in space.
  vec3 normal;              ///< Surface normal.
  vec2 uv;                  ///< U/V coordinate (for texture mapping).
};

} // namespace mageray

#endif // MAGERAY_HITINFO_H_
