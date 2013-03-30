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

#ifndef MAGERAY_TRACER_H_
#define MAGERAY_TRACER_H_

#include "base/platform.h"
#include "base/types.h"
#include "image.h"
#include "ray.h"
#include "vec.h"

namespace mageray {

class Scene;

/// Ray-tracing configuration parameters.
struct TraceConfig {
  /// Maximum number of trace recursions.
  unsigned max_recursions;

  /// Anti aliasing depth (0 = no anti aliasing).
  unsigned antialias_depth;
};

class Tracer {
  public:
    Tracer();

    /// Generate an image of the current scene.
    /// @param image The image to render to.
    void GenerateImage(Image& image) const;

    void SetScene(const Scene* scene) {
      m_scene = scene;
    }

    TraceConfig& Config() {
      return m_config;
    }

  private:
    struct TraceInfo {
      vec3 color;
      scalar alpha;
      scalar distance;
    };

    bool TraceRay(const Ray& ray, TraceInfo& info, const unsigned depth) const;

    TraceConfig m_config;

    const Scene* m_scene;

    FORBID_COPY(Tracer);
};

} // namespace mageray

#endif // MAGERAY_TRACER_H_
