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

#include "sampler.h"

#include "base/log.h"
#include "image.h"

namespace mageray {

// This is a fairly quick version of linear interpolation between two 32-bit
// colors using an 8-bit fractional weight (0-255). It uses semi-packed
// multiplication (two color components per multiplication), which means that
// only two integer multiplications are used in the operation.
static inline Pixel::Composite Lerp(const Pixel::Composite c1,
    const Pixel::Composite c2, const int w) {
  const Pixel::Composite c1_a = c1 & 0x00ff00ff;
  const Pixel::Composite c1_b = c1 & 0xff00ff00;
  const Pixel::Composite c2_a = c2 & 0x00ff00ff;
  const Pixel::Composite c2_b = c2 & 0xff00ff00;
  return ((((c1_a << 8) + (c2_a - c1_a) * w) >> 8) & 0x00ff00ff) |
      (((c1_b + ((c2_b >> 8) - (c1_b >> 8)) * w)) & 0xff00ff00);
}

Pixel Sampler::Sample(scalar s, scalar t) const {
  ASSERT(m_image, "No image.");

  // Convert floating point coordinates to fixed point coordinates (8-bit
  // sub-pixel precision).
  int s_fixed = static_cast<int>(s * m_image->m_s_scale);
  int t_fixed = static_cast<int>(t * m_image->m_t_scale);
  int sw = s_fixed & 0xff;
  int tw = t_fixed & 0xff;

  // Convert to pixel indices, taking repeating/clamping into account.
  int si, ti, si2, ti2;
  si = s_fixed >> 8;
  ti = t_fixed >> 8;
  if (m_repeat_s) {
    si = si % m_image->Width();
    si2 = (si + 1) % m_image->Width();
  } else {
    si = std::min(std::max(0, si), m_image->Width() - 1);
    si2 = std::min(si + 1, m_image->Width() - 1);
  }
  if (m_repeat_t) {
    ti = ti % m_image->Height();
    ti2 = (ti + 1) % m_image->Height();
  } else {
    ti = std::min(std::max(0, ti), m_image->Height() - 1);
    ti2 = std::min(ti + 1, m_image->Height() - 1);
  }

  ASSERT(si >= 0 && si < m_image->Width() && ti >= 0 && ti < m_image->Height(),
      "Indices out of range.");

  // Look up 4 neighbouring pixels.
  Pixel::Composite c1 = m_image->PixelAt(si, ti);
  Pixel::Composite c2 = m_image->PixelAt(si2, ti);
  Pixel::Composite c3 = m_image->PixelAt(si, ti2);
  Pixel::Composite c4 = m_image->PixelAt(si2, ti2);

  // Linear interpolation along s-axis.
  Pixel::Composite cs1 = Lerp(c1, c2, sw);
  Pixel::Composite cs2 = Lerp(c3, c4, sw);

  // Linear interpolation along t-axis.
  return Lerp(cs1, cs2, tw);
}

} // namespace mageray
