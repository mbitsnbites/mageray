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

#ifndef MAGERAY_BASE_TYPES_H_
#define MAGERAY_BASE_TYPES_H_

// Floating point type used throughout the program.
#if !defined(SCALAR_IS_FLOAT) && !defined(SCALAR_IS_DOUBLE)
# define SCALAR_IS_FLOAT
#endif

#ifdef SCALAR_IS_FLOAT
typedef float scalar;
#else
typedef double scalar;
#endif

// We need a very small distance at times (e.g. for checking if we're > 0, but
// accounting for numerical rounding errors).
#ifdef SCALAR_IS_FLOAT
# define EPSILON 1e-10f
#else
# define EPSILON 1e-50
#endif

// Precision dependent maximum distance.
#ifdef SCALAR_IS_FLOAT
# define MAX_DISTANCE 1e30f
#else
# define MAX_DISTANCE 1e300
#endif

// Since we simply can't seem to get PI into the C++ standard (?), put it here.
#ifdef SCALAR_IS_FLOAT
# define PI 3.1415926535897932384626433f
#else
# define PI 3.141592653589793238462643383279502884197169399375105820974944592308
#endif

// Convenience macro for disabling assignment and copying for a class.
#define FORBID_COPY(x)  \
  x(const x&) = delete; \
  void operator=(const x&) = delete

#endif // MAGERAY_BASE_TYPES_H_
