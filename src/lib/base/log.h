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

#ifndef RAYMAGE_BASE_LOG_H_
#define RAYMAGE_BASE_LOG_H_

#include <cstdio>
#include <iostream>
#ifdef _DEBUG
# include <cstdlib>
#endif

// Generic log macro (used by other log macros).
#define LOG_TO_STREAM(s, tag, msg, ...) { \
    char __buf[1024]; \
    __buf[1023] = 0; \
    std::snprintf(__buf, 1024, msg, ##__VA_ARGS__); \
    s << tag << __FILE__ << ":" << __LINE__ << " ("<< __FUNCTION__ << "): " \
        << __buf << std::endl; \
  }

// Log information to std out (debug & release).
#define LOG(msg, ...) LOG_TO_STREAM(std::cout, "[LOG] ", msg, ##__VA_ARGS__)

// Log debug information to std out (debug only).
#ifdef _DEBUG
# define DLOG(msg, ...) LOG_TO_STREAM(std::cout, "[DEBUG] ", msg, ##__VA_ARGS__)
#else
# define DLOG(msg, ...)
#endif

// Assert macro with log to std err (debug only).
#ifdef _DEBUG
# define ASSERT(x, msg, ...) if(!(x)) { \
    LOG_TO_STREAM(std::cerr, "[ASSERTION FAILED] ", msg, ##__VA_ARGS__); \
    std::abort(); \
  }
#else
# define ASSERT(x, msg, ...)
#endif

#endif // RAYMAGE_BASE_LOG_H_
