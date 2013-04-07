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

#ifndef MAGERAY_VEC_H_
#define MAGERAY_VEC_H_

#include <cmath>
#include <iostream>

#include "base/types.h"

namespace mageray {

/// Three component vector.
struct vec3 {
  scalar x;    ///< First component of the vector.
  scalar y;    ///< Second component of the vector.
  scalar z;    ///< Third component of the vector.

  enum Axis {
    X = 0,
    Y = 1,
    Z = 2
  };

  vec3() {}

  vec3(const scalar s) :
      x(s), y(s), z(s) {}

  vec3(const int s) :
      x(scalar(s)), y(scalar(s)), z(scalar(s)) {}

  vec3(const scalar x_, const scalar y_, const scalar z_) :
      x(x_), y(y_), z(z_) {}

  vec3(const int x_, const int y_, const int z_) :
      x(scalar(x_)), y(scalar(y_)), z(scalar(z_)) {}

#ifdef SCALAR_IS_FLOAT
  vec3(const double s) :
      x(scalar(s)), y(scalar(s)), z(scalar(s)) {}

  vec3(const double x_, const double y_, const double z_) :
      x(scalar(x_)), y(scalar(y_)), z(scalar(z_)) {}
#else
  vec3(const float s) :
      x(scalar(s)), y(scalar(s)), z(scalar(s)) {}

  vec3(const float x_, const float y_, const float z_) :
      x(scalar(x_)), y(scalar(y_)), z(scalar(z_)) {}
#endif

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

  /// Vector multiplication (element wise).
  vec3 operator*(const vec3& v) const {
    return vec3(x * v.x, y * v.y, z * v.z);
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

  /// Vector equality.
  bool operator==(const vec3& v) const {
    return x == v.x && y == v.y && z == v.z;
  }

  /// Vector inequality.
  bool operator!=(const vec3& v) const {
    return x != v.x || y != v.y || z != v.z;
  }

  /// Array access to the elements.
  /// @note There are no bounds checks!
  scalar& operator[](const int idx) {
    return (&x)[idx];
  }

  const scalar& operator[](const int idx) const {
    return (&x)[idx];
  }

  /// @returns The negative vector (scaled by -1).
  vec3 Neg() const {
    return vec3(-x, -y, -z);
  }

  /// @returns The element wise square root of the vector.
  vec3 Sqrt() const {
    return vec3(std::sqrt(x), std::sqrt(y), std::sqrt(z));
  }

  /// @returns The squared absolute value of the vector, |v|^2.
  scalar AbsSqr() const {
    return x * x + y * y + z * z;
  }

  /// @returns The absolute value of the vector, |v|.
  scalar Abs() const {
    return std::sqrt(AbsSqr());
  }

  /// @returns The dot product of two vectors.
  scalar Dot(const vec3& v) const {
    return x * v.x + y * v.y + z * v.z;
  }

  /// @returns The cross product of two vectors.
  vec3 Cross(const vec3& v) const {
    return vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }

  /// @returns The normalized vector.
  vec3 Normalize() const {
    scalar denom = this->AbsSqr();

    // Too small?
    if (denom < EPSILON) {
      return *this;
    }

    // Already normalized?
    scalar diff_one = denom - scalar(1.0);
    if (diff_one * diff_one < EPSILON) {
      return *this;
    }

    return *this * (scalar(1.0) / std::sqrt(denom));
  }

  /// @returns The next axis, modulo Z.
  static Axis NextAxis(const Axis a) {
    static const Axis next_axis[3] = {Y, Z, X};
    return next_axis[a];
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

  vec2(const scalar s) :
      u(s), v(s) {}

  vec2(const scalar u_, const scalar v_) :
      u(u_), v(v_) {}

#ifdef SCALAR_IS_FLOAT
  vec2(const double u_, const double v_) :
      u(scalar(u_)), v(scalar(v_)) {}
#else
  vec2(const float u_, const float v_) :
      u(scalar(u_)), v(scalar(v_)) {}
#endif

  vec2(const int u_, const int v_) :
      u(scalar(u_)), v(scalar(v_)) {}

  /// Vector addition.
  vec2 operator+(const vec2& other) const {
    return vec2(u + other.u, v + other.v);
  }
  /// Scalar multiplication.
  vec2 operator*(const scalar s) const {
    return vec2(s * u, s * v);
  }

  friend std::ostream& operator<<(std::ostream& os, const vec2& v) {
    os << "(" << v.u << "," << v.v << ")";
    return os;
  }
};

} // namespace mageray

#endif // MAGERAY_VEC_H_

