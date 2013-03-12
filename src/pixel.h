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

#ifndef MAGERAY_PIXEL_H_
#define MAGERAY_PIXEL_H_

#include <cstdint>
#include <ostream>

#include "base/log.h"
#include "vec.h"

/// A 32-bit integer pixel (ARGB).
/// A pixel is in the format ARGB (8 bits per component, 32 bits total) when
/// interpreted as Pixel::Composite (e.g. stored in a CPU register), but it is
/// stored in machine endian format in memory (i.e. B,G,R,A for little endian
/// machines and A,R,G,B for big endian machines).
class Pixel {
  public:
    /// A composite value of all color components.
    /// The Pixel class is essentially a wrapper around the opaque
    /// Pixel::Composite type. Outside of the Pixel class, the composite value
    /// type can be suitable for tight pixel processing code since it is a
    /// basic integer type that can be stored in CPU registers etc.
    typedef std::uint32_t Composite;

    Pixel() {}

    Pixel(const Composite& c)
        : m_composite(c) {}

    Pixel(const int r, const int g, const int b)
        : m_composite(ToComposite(r, g, b, 255)) {}

    Pixel(const int r, const int g, const int b, const int a)
        : m_composite(ToComposite(r, g, b, a)) {}

    Pixel(const scalar r, const scalar g, const scalar b)
        : m_composite(ToComposite(r, g, b, 1.0f)) {}

    Pixel(const scalar r, const scalar g, const scalar b, const scalar a)
        : m_composite(ToComposite(r, g, b, a)) {}

    Pixel(const vec3& v)
        : m_composite(ToComposite(v.x, v.y, v.z, 1.0f)) {}

    /// Red component.
    /// @returns The red component of the pixel (0-255).
    int r() const {
      return (m_composite >> 16) & 0xff;
    }

    /// Green component.
    /// @returns The green component of the pixel (0-255).
    int g() const {
      return (m_composite >> 8) & 0xff;
    }

    /// Blue component.
    /// @returns The blue component of the pixel (0-255).
    int b() const {
      return m_composite & 0xff;
    }

    /// Alpha component.
    /// @returns The alpha component of the pixel (0-255).
    int a() const {
      return m_composite >> 24;
    }

    operator Composite() const {
      return m_composite;
    }

    Pixel& operator=(const Composite& c) {
      m_composite = c;
      return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const Pixel& p) {
      os << "(" << p.r() << "," << p.g() << "," << p.b() << "," << p.a() << ")";
      return os;
    }

  private:
    static Composite ToComposite(const int r, const int g, const int b,
        const int a) {
      return b | (g << 8) | (r << 16) | (a << 24);
    }

    static Composite ToComposite(const scalar r, const scalar g, const scalar b,
        const scalar a) {
      return static_cast<int>(b * 255.0f) |
          (static_cast<int>(g * 255.0f) << 8) |
          (static_cast<int>(r * 255.0f) << 16) |
          (static_cast<int>(a * 255.0f) << 24);
    }

    Composite m_composite;
};

#endif // MAGERAY_PIXEL_H_
