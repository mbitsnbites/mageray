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

#ifndef MAGERAY_SAMPLER_H
#define MAGERAY_SAMPLER_H

#include "base/types.h"
#include "pixel.h"
#include "vec.h"

namespace mageray {

class Image;

class Sampler {
  public:
    Sampler() : m_image(NULL), m_repeat_s(true), m_repeat_t(true) {}

    void SetImage(const Image* image);

    void SetRepeat(bool repeat_s, bool repeat_t) {
      m_repeat_s = repeat_s;
      m_repeat_t = repeat_t;
    }

    bool HasImage() const {
      return m_image != NULL;
    }

    /// Sample a color value using bilinear interpolation.
    /// @param[in] s The s coordinate (0 = left, 1 = right).
    /// @param[in] t The t coordinate (0 = bottom, 1 = top).
    /// @returns An interpolated color value.
    Pixel Sample(const vec2& coord) const;

  private:
    const Image* m_image;

    float m_s_scale;  // Factor for converting s coordinates to fixed point.
    float m_t_scale;  // Factor for converting t coordinates to fixed point.

    bool m_repeat_s;
    bool m_repeat_t;
};

} // namespace mageray

#endif // MAGERAY_SAMPLER_H
