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

#include <mutex>

#include "base/platform.h"
#include "base/random.h"
#include "base/types.h"
#include "image.h"
#include "photon_map.h"
#include "ray.h"
#include "vec.h"

namespace mageray {

class Light;
class Scene;

/// Ray-tracing configuration parameters.
struct TraceConfig {
  /// Maximum number of trace recursions.
  unsigned max_recursions;

  /// Anti aliasing depth (0 = no anti aliasing).
  unsigned antialias_depth;

  /// Soft shadow recursion depth (0 = no soft shadows).
  unsigned soft_shadow_depth;

  /// Number of photons to use in the photon map (0 = no photon mapping).
  unsigned max_photons;

  /// Maximum recursion depth for tracing photons.
  unsigned max_photon_depth;
};

class Tracer {
  public:
    Tracer();

    /// Generate a photon map for the current scene.
    void GeneratePhotonMap();

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
    /// Thread coordination class.
    /// The thread controller is responsible for coordinating work packages
    /// between threads.
    class ThreadController {
      public:
        ThreadController(const int width, const int height);

        /// Get the next sub image area to render.
        /// @param[out] left Left of the sub image area.
        /// @param[out] top Top of the sub image area.
        /// @param[out] width The width of the sub image area.
        /// @param[out] height The height of the sub image area.
        /// @returns false if there is no more work to be done.
        bool NextArea(int& left, int& top, int& width, int& height);

      private:
        /// Block size (horizontal and vertical).
        static const int s_block_size = 32;

        const int m_width;
        const int m_height;
        int m_num_rows;
        int m_num_cols;

        std::mutex m_lock;
        int m_row;
        int m_col;
    };

    /// Worker method.
    /// Several threads will run this method simultaneously.
    /// @param controller The thread controller.
    /// @param image The target image.
    void DoWork(ThreadController* controller, Image* image) const;

    /// Result information for a single traced ray.
    struct TraceInfo {
      vec3 color;
      scalar alpha;
      scalar distance;
    };

    /// Check light contribution from a light source (0.0 - 1.0).
    scalar Shadow(const Light* light, const vec3& position) const;

    /// Trace a single ray.
    bool TraceRay(const Ray& ray, TraceInfo& info, const unsigned depth) const;

    /// Result information for a single traced photon.
    struct PhotonInfo {
      vec3 position;
      vec3 direction;
      vec3 color;
    };

    /// Trace a single photon.
    bool TracePhoton(const Ray& ray, PhotonInfo& info, Random& random,
        const unsigned depth, const vec3& color) const;

    TraceConfig m_config;

    const Scene* m_scene;

    PhotonMap m_photon_map;

    FORBID_COPY(Tracer);
};

} // namespace mageray

#endif // MAGERAY_TRACER_H_
