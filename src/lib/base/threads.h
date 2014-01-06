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

#ifndef MAGERAY_BASE_THREADS_H_
#define MAGERAY_BASE_THREADS_H_

#if defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
#include <process.h>
#else
#include <unistd.h>
#include <map>
#endif

namespace mageray {

class Thread {
  public:
    /// Get the hardware concurrency.
    /// This is mainly a replacement for std::thread::hardware_concurrency
    /// since g++ before 4.7 does not support it.
    static int hardware_concurrency() {
      int concurrency;
#if defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
      SYSTEM_INFO si;
      GetSystemInfo(&si);
      concurrency = (int) si.dwNumberOfProcessors;
#elif defined(_SC_NPROCESSORS_ONLN)
      concurrency = sysconf(_SC_NPROCESSORS_ONLN);
#elif defined(_SC_NPROC_ONLN)
      concurrency = sysconf(_SC_NPROC_ONLN);
#else
      concurrency = 0;
#endif
      // If we can't determine the concurrency (unlikely), just say "2".
      return concurrency > 0 ? concurrency : 2;
  }
};

} // namespace mageray

#endif // MAGERAY_BASE_THREADS_H_
