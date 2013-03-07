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

#include "mat.h"

mat3x4 mat3x4::operator*(const mat3x4& m) const {
  return mat3x4(
      // Row 1.
      m11 * m.m11 + m12 * m.m21 + m13 * m.m31,
      m11 * m.m12 + m12 * m.m22 + m13 * m.m32,
      m11 * m.m13 + m12 * m.m23 + m13 * m.m33,
      m11 * m.m14 + m12 * m.m24 + m13 * m.m34 + m14,

      // Row 2.
      m21 * m.m11 + m22 * m.m21 + m23 * m.m31,
      m21 * m.m12 + m22 * m.m22 + m23 * m.m32,
      m21 * m.m13 + m22 * m.m23 + m23 * m.m33,
      m21 * m.m14 + m22 * m.m24 + m23 * m.m34 + m24,

      // Row 3.
      m31 * m.m11 + m32 * m.m21 + m33 * m.m31,
      m31 * m.m12 + m32 * m.m22 + m33 * m.m32,
      m31 * m.m13 + m32 * m.m23 + m33 * m.m33,
      m31 * m.m14 + m32 * m.m24 + m33 * m.m34 + m34
  );
}

mat3x4 mat3x4::Inverse() const {
  mat3x4 inv;

  inv.m11 = m22 * m33 - m32 * m23;
  inv.m12 = m32 * m13 - m12 * m33;
  inv.m13 = m12 * m23 - m22 * m13;
  inv.m14 = m12 * m24 * m33 -
            m12 * m23 * m34 + 
            m22 * m13 * m34 - 
            m22 * m14 * m33 - 
            m32 * m13 * m24 + 
            m32 * m14 * m23;

  inv.m21 = m31 * m23 - m21 * m33;
  inv.m22 = m11 * m33 - m31 * m13;
  inv.m23 = m21 * m13 - m11 * m23;
  inv.m24 = m11 * m23 * m34 -
            m11 * m24 * m33 -
            m21 * m13 * m34 +
            m21 * m14 * m33 +
            m31 * m13 * m24 -
            m31 * m14 * m23;

  inv.m31 = m21 * m32 - m31 * m22;
  inv.m32 = m31 * m12 - m11 * m32;
  inv.m33 = m11 * m22 - m21 * m12;
  inv.m34 = m11 * m24 * m32 -
            m11 * m22 * m34 +
            m21 * m12 * m34 -
            m21 * m14 * m32 -
            m31 * m12 * m24 +
            m31 * m14 * m22;

  scalar det = m11 * inv.m11 + m12 * inv.m21 + m13 * inv.m31;
  ASSERT(det != 0.0f, "Singularity.");
  inv *= 1.0f / det;

  return inv;
}

