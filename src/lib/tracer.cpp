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

#include "tracer.h"

#include <list>
#include <thread>

#include "base/log.h"
#include "base/perf.h"
#include "hitinfo.h"
#include "light.h"
#include "material.h"
#include "pixel.h"
#include "scene.h"
#include "shader.h"

namespace mageray {

Tracer::ThreadController::ThreadController(const int width, const int height) :
    m_width(width), m_height(height), m_row(0), m_col(0) {
  // Calculate number of sub-blocks.
  m_num_rows = (height + s_block_size - 1) / s_block_size;
  m_num_cols = (width + s_block_size - 1) / s_block_size;
}

bool Tracer::ThreadController::NextArea(int& u0, int& v0, int& width,
    int& height) {
  m_lock.lock();

  bool do_work = (m_row < m_num_rows);
  if (do_work) {
    // Region for this sub image.
    u0 = m_col * s_block_size;
    width = std::min(u0 + s_block_size, m_width) - u0;
    v0 = m_row * s_block_size;
    height = std::min(v0 + s_block_size, m_height) - v0;

    // Next sub image.
    ++m_col;
    if (m_col == m_num_cols) {
      m_col = 0;
      ++m_row;
    }
  }

  m_lock.unlock();

  return do_work;
}

Tracer::Tracer() : m_scene(NULL) {
  // Default configuration.
  m_config.max_recursions = 4;
  m_config.antialias_depth = 3;
}

void Tracer::DoWork(ThreadController* controller, Image* image) const {
  // Set up camera.
  vec3 cam_pos = m_scene->m_camera.Position();
  vec3 forward = m_scene->m_camera.Forward();
  vec3 right = m_scene->m_camera.Right();
  vec3 up = m_scene->m_camera.Up();

  scalar img_width = static_cast<scalar>(image->Width());
  scalar img_height = static_cast<scalar>(image->Height());
  vec3 u_step = right * (scalar(1.0) / img_height);
  vec3 v_step = up * (scalar(-1.0) / img_height);

  // As long as there is work to do...
  int u0, v0, width, height;
  while (controller->NextArea(u0, v0, width, height)) {
    // Loop over rows.
    for (int  j = 0; j < height; ++j) {
      int v = v0 + j;
      vec3 dir = forward +
          u_step * (static_cast<scalar>(u0) - scalar(0.5) * img_width) +
          v_step * (static_cast<scalar>(v) - scalar(0.5) * img_height);

      // Loop over columns in the row.
      for (int  i = 0; i < width; ++i) {
        int u = u0 + i;

        // Construct a ray.
        Ray ray(cam_pos, dir);

        // Trace a ray into the scene.
        Pixel result(0);
        TraceInfo info;
        if (TraceRay(ray, info, 0)) {
          result = Pixel(std::min(scalar(1.0), info.color.x),
                         std::min(scalar(1.0), info.color.y),
                         std::min(scalar(1.0), info.color.z),
                         info.alpha);
        }
        image->PixelAt(u, v) = result;

        dir += u_step;
      }
    }
  }
}

void Tracer::GenerateImage(Image& image) const {
  ASSERT(m_scene, "The scene is undefined.");
  ScopedPerf _raytrace = ScopedPerf("Raytrace image");

  // Get level of hardware concurrency.
  int concurrency = std::thread::hardware_concurrency();
  if (concurrency == 0) {
    // NOTE: hardware_concurrency() in gcc 4.6.3 always returns zero :(
    concurrency = 2;
  }

  // Start threads.
  ThreadController controller(image.Width(), image.Height());
  std::list<std::thread> threads;
  for (int i = 0; i < concurrency; ++i) {
    threads.push_back(std::thread(&Tracer::DoWork, this, &controller, &image));
  }

  // Wait for threads to finish.
  for (auto it = threads.begin(); it != threads.end(); it++) {
    it->join();
  }

  _raytrace.Done();
}

bool Tracer::TraceRay(const Ray& ray, TraceInfo& info, const unsigned depth)
    const {
  if (depth > m_config.max_recursions) {
    return false;
  }

  // Intersect scene.
  HitInfo hit = HitInfo::CreateNoHit();
  if (!m_scene->m_object_tree.Intersect(ray, hit)) {
    return false;
  }

  // Get surface properties.
  hit.object->CompleteHitInfo(ray, hit);

  // Startup info, before shading.
  info.color = vec3(0);
  info.alpha = scalar(1.0);
  info.distance = hit.t;

  // Get material.
  const Material* material = hit.object->Material();

  // Fallback to surface normal visualization if we don't have a material.
  if (UNLIKELY(!material || !material->Shader())) {
    info.color = vec3(0.5) + (hit.normal * scalar(0.5));
    return true;
  }

  // View direction.
  vec3 view_dir = (hit.point - ray.Origin()).Normalize();

  // Get the shader for this material.
  const Shader* shader = material->Shader();
  ASSERT(shader, "Missing shader.");

  Shader::SurfaceParam surface_param;
  surface_param.material = material;
  surface_param.position = hit.point;
  surface_param.normal = hit.normal;
  surface_param.uv = hit.uv;
  surface_param.view_dir = view_dir;

  // Get material properties.
  Shader::MaterialParam material_param;
  shader->MaterialPass(surface_param, material_param);

  // Reflection?
  vec3 mirror = material_param.specular * material->Mirror();
  if (mirror != vec3(0.0)) {
    // Reflected direction.
    vec3 reflect_dir = ray.Direction() -
        hit.normal * (scalar(2.0) * hit.normal.Dot(ray.Direction()));

    // Nudge origin point in order to avoid re-intersecting the origin surface.
    // TODO(mage): The "nudge distance" should be relative to object scale
    // somehow.
    vec3 reflect_start = hit.point + reflect_dir * scalar(0.0001);

    // Trace ray.
    Ray reflect_ray(reflect_start, reflect_dir);
    TraceInfo reflect_info;
    if (TraceRay(reflect_ray, reflect_info, depth + 1)) {
      info.color += reflect_info.color * mirror;
    }
  }

  // Transparency?
  if (material_param.alpha < scalar(1.0)) {
    // Refracted direction.
    // TODO(mage): Implement me!
    vec3 refract_dir = ray.Direction();

    // Nudge origin point in order to avoid re-intersecting the origin surface.
    // TODO(mage): The "nudge distance" should be relative to object scale
    // somehow.
    vec3 refract_start = hit.point + refract_dir * scalar(0.0001);

    // Trace ray.
    scalar opacity = scalar(1.0) - material_param.alpha;
    Ray refract_ray(refract_start, refract_dir);
    TraceInfo refract_info;
    if (TraceRay(refract_ray, refract_info, depth + 1)) {
      info.color += refract_info.color * material_param.alpha;
      opacity *= scalar(1.0) - refract_info.alpha;
    }

    info.alpha = scalar(1.0) - opacity;
  }

  // Light contribution.
  vec3 light_contrib(0);
  if (material_param.diffuse != vec3(0.0) ||
      material_param.specular != vec3(0.0)) {
    // Iterate all the lights in the scene.
    for (auto it = m_scene->m_lights.begin(); it != m_scene->m_lights.end();
        it++) {
      Light* light = it->get();

      // Direction and distance to the light.
      vec3 light_dir = light->Position() - hit.point;
      scalar light_dist = light_dir.Abs();
      light_dir = light_dir * (scalar(1.0) / light_dist);

      // Scale light distance in order to avoid re-intersecting the origin
      // surface.
      light_dist *= scalar(0.9999);

      scalar cos_alpha = light_dir.Dot(hit.normal);
      if (cos_alpha > scalar(0.0)) {
        Shader::LightParam light_param;
        light_param.light = light;
        light_param.dir = light_dir;
        light_param.dist = light_dist;
        light_param.cos_alpha = cos_alpha;
        light_param.amount = scalar(1.0);

        // Determine light visibility factor (0.0 for completely shadowed).
        HitInfo shadow_hit = HitInfo::CreateShadowTest(light_dist);
        Ray shadow_ray(light->Position(), light_dir.Neg());
        if (m_scene->m_object_tree.Intersect(shadow_ray, shadow_hit)) {
          light_param.amount = scalar(0.0);
        }

        // Run per-light shader.
        light_contrib += shader->LightPass(surface_param, material_param,
            light_param);
      }
    }
  }

  // Run final shader pass.
  info.color += shader->FinalPass(surface_param, material_param, light_contrib);

  return true;
}

} // namespace mageray
