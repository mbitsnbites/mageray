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

#ifndef MAGERAY_MAT_H_
#define MAGERAY_MAT_H_

#include <cmath>
#include <iostream>

#include "base/log.h"
#include "vec.h"

/// 3x4 matrix.
class mat3x4 {
  public:
    mat3x4() {}

    mat3x4(const scalar s) :
        m11(s), m12(s), m13(s), m14(s),
        m21(s), m22(s), m23(s), m24(s),
        m31(s), m32(s), m33(s), m34(s) {}

    mat3x4(const scalar m11_, const scalar m12_, const scalar m13_, const scalar m14_,
           const scalar m21_, const scalar m22_, const scalar m23_, const scalar m24_,
           const scalar m31_, const scalar m32_, const scalar m33_, const scalar m34_) :
        m11(m11_), m12(m12_), m13(m13_), m14(m14_),
        m21(m21_), m22(m22_), m23(m23_), m24(m24_),
        m31(m31_), m32(m32_), m33(m33_), m34(m34_) {}

    /// Construct a matrix from a coordinate system.
    /// @param e1 Basis vector 1 (the "right" axis).
    /// @param e2 Basis vector 2 (the "forward" axis).
    /// @param e3 Basis vector 3 (the "up" axis).
    /// @param t Translation.
    mat3x4(const vec3& e1, const vec3& e2, const vec3& e3, const vec3& t) :
        m11(e1.x), m12(e2.x), m13(e3.x), m14(t.x),
        m21(e1.y), m22(e2.y), m23(e3.y), m24(t.y),
        m31(e1.z), m32(e2.z), m33(e3.z), m34(t.z) {}


    /// Create an identity matrix.
    static mat3x4 Identity() {
      return mat3x4(
          1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0
      );
    }

    /// Create a scaling matrix.
    static mat3x4 Scale(const scalar s) {
      return mat3x4(
          s, 0, 0, 0,
          0, s, 0, 0,
          0, 0, s, 0
      );
    }

    /// Create a translation matrix.
    static mat3x4 Translate(const scalar x, const scalar y, const scalar z) {
      return mat3x4(
          1, 0, 0, x,
          0, 1, 0, y,
          0, 0, 1, z
      );
    }

    /// Create a translation matrix.
    static mat3x4 Translate(const vec3& t) {
      return mat3x4(
          1, 0, 0, t.x,
          0, 1, 0, t.y,
          0, 0, 1, t.z
      );
    }

    /// Matrix * matrix multiplication.
    /// In this multiplication operation, each matrix is interpreted as a 4x4
    /// matrix, where the last row equals (0, 0, 0, 1), and the result is
    /// truncated back to a 3x4 matrix.
    mat3x4 operator*(const mat3x4& m) const;

    /// Matrix * scalar multiplication.
    mat3x4 operator*(const scalar s) const {
      return mat3x4(
          m11 * s, m12 * s, m13 * s, m14 * s,
          m21 * s, m22 * s, m23 * s, m24 * s,
          m31 * s, m32 * s, m33 * s, m34 * s
      );
    }

    /// Unary matrix * scalar multiplication.
    mat3x4& operator*=(const scalar s) {
      m11 *= s; m12 *= s; m13 *= s; m14 *= s;
      m21 *= s; m22 *= s; m23 *= s; m24 *= s;
      m31 *= s; m32 *= s; m33 *= s; m34 *= s;
      return *this;
    }

    /// Inverse matrix.
    /// In this operation, the matrix is interpreted as a 4x4 matrix, where the
    /// last row equals (0, 0, 0, 1), and the result is truncated back to a 3x4
    /// matrix.
    mat3x4 Inverse() const;

    /// Matrix * vector multiplication (transformation).
    /// In this operation, the vector is interpreted as a four element vector,
    /// where the last element equals 1, and the result is truncated back to a
    /// three element vector.
    vec3 operator*(const vec3& v) const {
      return vec3(
          m11 * v.x + m12 * v.y + m13 * v.z + m14,
          m21 * v.x + m22 * v.y + m23 * v.z + m24,
          m31 * v.x + m32 * v.y + m33 * v.z + m34
      );
    }

    /// Transform point.
    vec3 TransformPoint(const vec3& p) const {
      return *this * p;
    }

    /// Transform direction (no translation).
    vec3 TransformDirection(const vec3& n) const {
      return vec3(
          m11 * n.x + m12 * n.y + m13 * n.z,
          m21 * n.x + m22 * n.y + m23 * n.z,
          m31 * n.x + m32 * n.y + m33 * n.z
      );
    }

    friend std::ostream& operator<<(std::ostream& os, const mat3x4& m) {
      os << std::endl <<
          "|" << m.m11 << "," << m.m12 << "," << m.m13 << "," << m.m14 << "|" << std::endl <<
          "|" << m.m21 << "," << m.m22 << "," << m.m23 << "," << m.m24 << "|" << std::endl <<
          "|" << m.m31 << "," << m.m32 << "," << m.m33 << "," << m.m34 << "|";
      return os;
    }

  private:
    // Matrix elements
    scalar m11, m12, m13, m14;
    scalar m21, m22, m23, m24;
    scalar m31, m32, m33, m34;
};

#endif // MAGERAY_VEC_H_

