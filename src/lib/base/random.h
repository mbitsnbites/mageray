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

#ifndef MAGERAY_BASE_RANDOM_H_
#define MAGERAY_BASE_RANDOM_H_

#include <random>

#include "base/types.h"

namespace mageray {

/// Mersenne twister pseudo random number generator.
/// This is fairly similar to std::mt19937 (C++11), and the implementation is
/// based on the pseudo code fond at Wikipedia
/// (http://en.wikipedia.org/wiki/Mersenne_twister). It's not as fast as
/// std::mt19937 though (g++ v4.6).
class MersenneTwister {
  public:
    MersenneTwister() {
      seed();
    }

    MersenneTwister(const unsigned s) {
      seed(s);
    }

    /// Seed the random number generator.
    /// Re-initializes the internal state sequence to pseudo-random values by
    /// applying a linear random generator on a single value.
    /// @param s The seed.
    void seed(const unsigned s = 5489) {
      unsigned x = s;
      for (unsigned i = 0; i < 624; ++i) {
        m_numbers[i] = x;
        x = 1812433253 * (x ^ (x >> 30)) + 1;
      }
      m_index = 0;
    }

    /// @returns a new random number.
    unsigned operator()() {
      m_index = (m_index + 1) % 624;
      unsigned i = m_index;

      // Generate output.
      unsigned y = m_numbers[i];
      y = y ^ (y >> 11);
      y = y ^ ((y << 7) & 2636928640);
      y = y ^ ((y << 15) & 4022730752);
      y = y ^ (y >> 17);

      // Update array.
      unsigned x = (m_numbers[i] & 0x80000000) +
          (m_numbers[(i + 1) % 624] & 0x7fffffff);
      unsigned z = m_numbers[(i + 397) % 624] ^ (x >> 1);
      m_numbers[i] = x & 1 ? z ^ 2567483615 : z;

      return y;
    }

  private:
    unsigned m_numbers[624];
    unsigned m_index;
};

/// Random number generation template class.
/// Random numbers can be generated for a variety of different types.
///
/// This random number generator wrapper works with random number generator
/// classes that generate 32-bit unsigned integers, and have the following
/// methods: seed(unsigned), unsigned operator()(). This includes std::mt19937
/// and MersenneTwister, for instance.
template <class R>
class Random {
  public:
    /// Set the seed for the random number generator.
    /// @param s The seed.
    void Seed(const unsigned s) {
      m_random.seed(s);
    }

    /// @returns A 32-bit signed integer in the range [-2^31, 2^31-1].
    int Int() {
      return static_cast<int>(Unsigned());
    }

    /// @returns A 32-bit unsigned integer in the range [0, 2^32-1].
    unsigned Unsigned() {
      return m_random();
    }

    /// @returns A 32-bit floating point value in the range (0.0, 1.0).
    float Float() {
      uint32_t r = Unsigned();
      if (!r) {
        return 0.0f;
      }
      return AsFloat(((0x7e - LeadingZeros(r)) << 23) | r & 0x007fffff);
    }

    /// @returns A 32-bit floating point value in the range (-1.0, 1.0).
    float SignedFloat() {
      uint32_t r = Unsigned();
      if (!r) {
        return 0.0f;
      }
      return AsFloat((r << 31) | ((0x7e - LeadingZeros(r)) << 23) |
          r & 0x007fffff);
    }

    /// @returns A scalar value in the range (0.0, 1.0).
    scalar Scalar() {
      return scalar(Float());
    }

    /// @returns A scalar value in the range (-1.0, 1.0).
    scalar SignedScalar() {
      return scalar(SignedFloat());
    }

    /// @returns A vec3 with random elements in the range (0.0, 1.0).
    vec3 Vec3() {
      return vec3(Scalar(), Scalar(), Scalar());
    }

    /// @returns A vec3 with random elements in the range (-1.0, 1.0).
    vec3 SignedVec3() {
      return vec3(SignedScalar(), SignedScalar(), SignedScalar());
    }

  private:
    /// Count leading zero bits.
    /// @param x The unsigned integer to count leading zero bits in.
    /// @note Undefined if x is zero.
    static uint32_t LeadingZeros(uint32_t x) {
#if defined(__GNUC__)
      return __builtin_clz(x);
#else
      // Robert Harley's algorithm
      static const char table[64] =
          {32, 31, 0, 16, 0, 30, 3, 0, 15, 0, 0, 0, 29, 10, 2, 0,
           0, 0, 12, 14, 21, 0, 19, 0, 0, 28, 0, 25, 0, 9, 1, 0,
           17, 0, 4, 0, 0, 0, 11, 0, 13, 22, 20, 0, 26, 0, 0, 18,
           5, 0, 0, 23, 0, 27, 0, 6, 0, 24, 7, 0, 8, 0, 0, 0};
       x = x | (x >> 1);
       x = x | (x >> 2);
       x = x | (x >> 4);
       x = x | (x >> 8);
       x = x | (x >>16);
       x = x * 0x06EB14F9;
       return table[x >> 26];
#endif
    }

    /// Interprests an integer as a 32-bit IEEE 794 floating point value.
    static float AsFloat(uint32_t x) {
      union {
        uint32_t i;
        float f;
      } u;
      u.i = x;
      return u.f;
    }

    // Random number generator.
    R m_random;
};

} // namespace mageray

#endif // MAGERAY_BASE_RANDOM_H_

