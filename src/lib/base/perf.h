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

#ifndef MAGERAY_BASE_PERF_H_
#define MAGERAY_BASE_PERF_H_

namespace mageray {

class Perf {
  public:
    static double GetTime();
    static void LogDelta(const char* label, const double& time);
};

class ScopedPerf {
  public:
    ScopedPerf(const char* label) : m_label(label), m_done(false) {
      m_start = Perf::GetTime();
    }

    ~ScopedPerf() {
      Done();
    }

    void Done() {
      if (m_done)
        return;
      Perf::LogDelta(m_label, Perf::GetTime() - m_start);
      m_done = true;
    }

  private:
    const char* m_label;
    bool m_done;
    double m_start;
};

class CumulativePerf {
  public:
    CumulativePerf(const char* label) : m_label(label), m_total(0.0), m_count(0) {}

    void Start() {
      m_start = Perf::GetTime();
    }

    void Stop() {
      m_total += Perf::GetTime() - m_start;
      m_count++;
    }

    void Report() const {
      Perf::LogDelta(m_label, m_total / static_cast<double>(m_count));
    }

  private:
    const char* m_label;
    double m_total;
    int m_count;
    double m_start;
};

} // namespace mageray

#endif // MAGERAY_BASE_PERF_H_
