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

#ifndef RAYMAGE_VEC_H_
#define RAYMAGE_VEC_H_

#include <cmath>
#include <iostream>

#include "base/types.h"

/// Three component vector.
struct vec3 {
  scalar x;    ///< First component of the vector.
  scalar y;    ///< Second component of the vector.
  scalar z;    ///< Third component of the vector.

  vec3() {}

  vec3(const scalar s) :
      x(s), y(s), z(s) {}

  vec3(const scalar x_, const scalar y_, const scalar z_) :
      x(x_), y(y_), z(z_) {}

  /// Vector addition.
  vec3 operator+(const vec3& v) const {
    return vec3(x + v.x, y + v.y, z + v.z);
  }

  /// Unary vector addition.
  vec3& operator+=(const vec3& v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  /// Vector subtraction.
  vec3 operator-(const vec3& v) const {
    return vec3(x - v.x, y - v.y, z - v.z);
  }

  /// Unary vector subtraction.
  vec3& operator-=(const vec3& v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  /// Scalar multiplication.
  vec3 operator*(const scalar s) const {
    return vec3(s * x, s * y, s * z);
  }

  /// Unary scalar multiplication.
  vec3& operator*=(const scalar s) {
    x *= s;
    y *= s;
    z *= s;
    return *this;
  }

  /// @returns The negative vector (scaled by -1).
  vec3 neg() const {
    return vec3(-x, -y, -z);
  }

  /// @returns The squared absolute value of the vector, |v|^2.
  scalar abs_sqr() const {
    return x * x + y * y + z * z;
  }

  /// @returns The absolute value of the vector, |v|.
  scalar abs() const {
    return std::sqrt(abs_sqr());
  }

  /// @returns The dot product of two vectors.
  scalar dot(const vec3& v) const {
    return x * v.x + y * v.y + z * v.z;
  }

  /// @returns The cross product of two vectors.
  vec3 cross(const vec3& v) const {
    return vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }

  friend std::ostream& operator<<(std::ostream& os, const vec3& v) {
    os << "(" << v.x << "," << v.y << "," << v.z << ")";
    return os;
  }
};

/// Two component vector.
struct vec2 {
  scalar u;    ///< First component of the vector.
  scalar v;    ///< Second component of the vector.

  vec2() {}

  vec2(const scalar u_, const scalar v_) :
      u(u_), v(v_) {}

  friend std::ostream& operator<<(std::ostream& os, const vec2& v) {
    os << "(" << v.u << "," << v.v << ")";
    return os;
  }
};

#endif // RAYMAGE_VEC_H_

