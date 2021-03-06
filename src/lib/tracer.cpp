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

#include <algorithm>
#include <cstdlib>
#include <list>
#include <thread>

#include "base/log.h"
#include "base/perf.h"
#include "base/random.h"
#include "base/threads.h"
#include "hitinfo.h"
#include "light.h"
#include "material.h"
#include "pixel.h"
#include "scene.h"
#include "shader.h"

namespace mageray {

//------------------------------------------------------------------------------
// Soft shadowing.
//------------------------------------------------------------------------------

/// Class for calculating soft shadows.
class SoftShadower {
  public:
    SoftShadower(const ObjectTree& tree, const Light* light,
        const vec3& position, const int count);

    scalar TotalUnblocked();

  private:
    enum OcclusionStatus {
      Untested = 0,
      Blocked = 1,
      Unblocked = 2
    };

    void RecShadow(int u1, int u2, int v1, int v2, int level);

    const ObjectTree* m_tree;
    const Light* m_light;
    const vec3 m_position;
    const int m_count;

    vec3 m_e1;
    vec3 m_e2;
    scalar m_step;

    std::vector<OcclusionStatus> m_occlusion;
};

SoftShadower::SoftShadower(const ObjectTree& tree, const Light* light,
    const vec3& position, const int count) :
    m_tree(&tree),
    m_light(light),
    m_position(position),
    m_count(count) {
  // Vector from the light source to the surface point.
  vec3 light_vec = position - light->Position();

  // Construct a size x size square facing towards the surface point.
  if (std::abs(light_vec.x) < std::abs(light_vec.y)) {
    m_e1 = light_vec.Cross(vec3(1,0,0));
  } else {
    m_e1 = light_vec.Cross(vec3(0,1,0));
  }
  m_e2 = light_vec.Cross(m_e1);
  m_e1 = m_e1 * (light->Size() / m_e1.Abs());
  m_e2 = m_e2 * (light->Size() / m_e2.Abs());

  // Step size along e1 and e2 vectors.
  m_step = scalar(1.0) / static_cast<scalar>(count - 1);
}

scalar SoftShadower::TotalUnblocked() {
  // Start with an undefined occlussion matrix.
  m_occlusion.resize(m_count * m_count);
  std::fill(m_occlusion.begin(), m_occlusion.end(), Untested);

  // Recursively check light occlusion.
  RecShadow(0, m_count - 1, 0, m_count - 1, 1);

  // Check how many rays were unblocked.
  int num_unblocked = 0;
  for (auto it = m_occlusion.begin(); it != m_occlusion.end(); it++) {
    if (*it == Unblocked) {
      num_unblocked++;
    }
  }

  return static_cast<scalar>(num_unblocked) /
      static_cast<scalar>(m_occlusion.size());
}

void SoftShadower::RecShadow(int u1, int u2, int v1, int v2, int level) {
  // Occlusion matrix indices.
  int idx1 = u1 + v1 * m_count;
  int idx2 = u2 + v1 * m_count;
  int idx3 = u1 + v2 * m_count;
  int idx4 = u2 + v2 * m_count;

  // 1st corner.
  if (m_occlusion[idx1] == Untested) {
    vec3 pos = m_light->Position() +
        m_e1 * (static_cast<scalar>(u1) * m_step - scalar(0.5)) +
        m_e2 * (static_cast<scalar>(v1) * m_step - scalar(0.5));
    HitInfo shadow_hit = HitInfo::CreateShadowTest(scalar(0.9999));
    Ray shadow_ray(pos, m_position - pos);
    bool occluded = m_tree->Intersect(shadow_ray, shadow_hit);
    m_occlusion[idx1] = occluded ? Blocked : Unblocked;
  }

  // 2nd corner.
  if (m_occlusion[idx2] == Untested) {
    vec3 pos = m_light->Position() +
        m_e1 * (static_cast<scalar>(u2) * m_step - scalar(0.5)) +
        m_e2 * (static_cast<scalar>(v1) * m_step - scalar(0.5));
    HitInfo shadow_hit = HitInfo::CreateShadowTest(scalar(0.9999));
    Ray shadow_ray(pos, m_position - pos);
    bool occluded = m_tree->Intersect(shadow_ray, shadow_hit);
    m_occlusion[idx2] = occluded ? Blocked : Unblocked;
  }

  // 3rd corner.
  if (m_occlusion[idx3] == Untested) {
    vec3 pos = m_light->Position() +
        m_e1 * (static_cast<scalar>(u1) * m_step - scalar(0.5)) +
        m_e2 * (static_cast<scalar>(v2) * m_step - scalar(0.5));
    HitInfo shadow_hit = HitInfo::CreateShadowTest(scalar(0.9999));
    Ray shadow_ray(pos, m_position - pos);
    bool occluded = m_tree->Intersect(shadow_ray, shadow_hit);
    m_occlusion[idx3] = occluded ? Blocked : Unblocked;
  }

  // 4th corner.
  if (m_occlusion[idx4] == Untested) {
    vec3 pos = m_light->Position() +
        m_e1 * (static_cast<scalar>(u2) * m_step - scalar(0.5)) +
        m_e2 * (static_cast<scalar>(v2) * m_step - scalar(0.5));
    HitInfo shadow_hit = HitInfo::CreateShadowTest(scalar(0.9999));
    Ray shadow_ray(pos, m_position - pos);
    bool occluded = m_tree->Intersect(shadow_ray, shadow_hit);
    m_occlusion[idx4] = occluded ? Blocked : Unblocked;
  }

  // Was this the last recursion level?
  if (u1 + 1 == u2) {
    return;
  }

  // Same result for all four corners (and at least recursion level 2)?
  OcclusionStatus status = m_occlusion[idx1];
  if (level >= 2 && m_occlusion[idx2] == status &&
      m_occlusion[idx3] == status && m_occlusion[idx4] == status) {
    // Fill inner area with the result from the four corners.
    for (int v = v1; v <= v2; ++v) {
      OcclusionStatus* ptr = &m_occlusion[v * m_count + u1];
      for (int u = u1; u <= u2; ++u) {
        *ptr++ = status;
      }
    }
  } else {
    // Recurse further...
    int u_mid = (u1 + u2) >> 1;
    int v_mid = (v1 + v2) >> 1;
    RecShadow(u1, u_mid, v1, v_mid, level + 1);
    RecShadow(u_mid, u2, v1, v_mid, level + 1);
    RecShadow(u1, u_mid, v_mid, v2, level + 1);
    RecShadow(u_mid, u2, v_mid, v2, level + 1);
  }
}


//------------------------------------------------------------------------------
// Threaded ray tracer.
//------------------------------------------------------------------------------

Tracer::ThreadController::ThreadController(const int width, const int height) :
    m_width(width), m_height(height), m_row(0), m_col(0) {
  // Calculate number of sub-blocks.
  m_num_rows = (height + s_block_size - 1) / s_block_size;
  m_num_cols = (width + s_block_size - 1) / s_block_size;
}

bool Tracer::ThreadController::NextArea(int& left, int& top, int& width,
    int& height) {
  m_lock.lock();

  // Region for this sub image.
  left = m_col * s_block_size;
  width = std::min(left + s_block_size, m_width) - left;
  top = m_row * s_block_size;
  height = std::min(top + s_block_size, m_height) - top;

  bool do_work = (m_row < m_num_rows);
  if (do_work) {
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


//------------------------------------------------------------------------------
// Ray tracer.
//------------------------------------------------------------------------------

Tracer::Tracer() : m_scene(NULL) {}

void Tracer::DoWork(ThreadController* controller, Image* image) const {
  // Set up camera.
  vec3 cam_pos = m_scene->Camera().Position();
  vec3 forward = m_scene->Camera().Forward();
  vec3 right = m_scene->Camera().Right();
  vec3 up = m_scene->Camera().Up();

  scalar img_width = static_cast<scalar>(image->Width());
  scalar img_height = static_cast<scalar>(image->Height());
  vec3 u_step = right * (scalar(1.0) / img_height);
  vec3 v_step = up * (scalar(-1.0) / img_height);

  // We need a random number generator for the jittering.
  Random rand;

  // As long as there is work to do...
  int left, top, width, height;
  while (controller->NextArea(left, top, width, height)) {
    // Loop over rows.
    int v0 = top;
    for (int  j = 0; j < height; ++j) {
      int v = v0 + j;

      // Every second row we do right to left.
      int u0 = j & 1 ? left + width - 1 : left;

      // Nominal ray direction for the first/last pixel in the row.
      vec3 dir0 = forward +
          u_step * (static_cast<scalar>(u0) - scalar(0.5) * img_width) +
          v_step * (static_cast<scalar>(v) - scalar(0.5) * img_height);

      // Loop over columns in the row.
      for (int  i = 0; i < width; ++i) {
        int u = j & 1 ? u0 - i : u0 + i;

        // Perform antialiasing loop (use several rays per pixel).
        TraceInfo min, max;
        TraceInfo sum;
        unsigned num_rays = 0;
        while (num_rays < m_scene->Config().rays_per_pixel) {
          // Jitter the ray direction.
          vec3 dir;
          if (m_scene->Config().rays_per_pixel > 1) {
            scalar rnd_u = rand.Scalar() - scalar(0.5);
            scalar rnd_v = rand.Scalar() - scalar(0.5);
            dir = dir0 + u_step * rnd_u + v_step * rnd_v;
          }
          else {
            dir = dir0;
          }

          // Construct a ray.
          Ray ray(cam_pos, dir);

          // Trace a ray into the scene.
          TraceInfo info;
          if (TraceRay(ray, info, 0)) {
            sum.color += info.color;
            sum.alpha += info.alpha;
          }
          num_rays++;

          // Determine min/max.
          if (num_rays == 1) {
            min = max = info;
          }
          else if (num_rays <= m_scene->Config().min_rays_per_pixel) {
            min.color = vec3::Min(min.color, info.color);
            min.alpha = std::min(min.alpha, info.alpha);
            max.color = vec3::Max(max.color, info.color);
            max.alpha = std::max(min.alpha, info.alpha);

            // Check if we should bail out.
            if (num_rays == m_scene->Config().min_rays_per_pixel) {
              static const scalar threshold(0.02);
              scalar delta = max.color.x - min.color.x +
                             max.color.y - min.color.y +
                             max.color.z - min.color.z +
                             max.alpha - min.alpha;
              if (delta < threshold) {
                break;
              }
            }
          }
        }

        // Construct final pixel color.
        const scalar scale = scalar(1.0) / scalar(num_rays);
        image->PixelAt(u, v) = Pixel(std::min(scalar(1.0), sum.color.x * scale),
                                     std::min(scalar(1.0), sum.color.y * scale),
                                     std::min(scalar(1.0), sum.color.z * scale),
                                     sum.alpha * scale);

        // Nominal direction for the next pixel in the row.
        if (j & 1) {
          dir0 -= u_step;
        } else {
          dir0 += u_step;
        }
      }
    }
  }
}

void Tracer::GenerateImage(Image* image) const {
  ASSERT(m_scene, "The scene is undefined.");
  ScopedPerf _raytrace = ScopedPerf("Raytrace image");

  // Get level of hardware concurrency.
  int concurrency = Thread::hardware_concurrency();
  DLOG("Using %d threads to render image.", concurrency);

  // Start threads.
  ThreadController controller(image->Width(), image->Height());
  std::list<std::thread> threads;
  for (int i = 0; i < concurrency; ++i) {
    threads.push_back(std::thread(&Tracer::DoWork, this, &controller, image));
  }

  // Wait for threads to finish.
  for (auto it = threads.begin(); it != threads.end(); it++) {
    it->join();
  }

  _raytrace.Done();
}

scalar Tracer::Shadow(const Light* light, const vec3& position) const {
  if (light->Size() > EPSILON && m_scene->Config().soft_shadow_depth > 0) {
    // Number of rays along each side of the light square.
    int count = (1 << m_scene->Config().soft_shadow_depth) + 1;

    // Compute light contribution using soft shadows.
    SoftShadower soft_shadower(m_scene->ObjectTree(), light, position, count);
    return soft_shadower.TotalUnblocked();
  } else {
    // No soft shadow, just use a single ray.
    vec3 light_vec = position - light->Position();
    HitInfo shadow_hit = HitInfo::CreateShadowTest(scalar(0.9999));
    Ray shadow_ray(light->Position(), light_vec);
    if (m_scene->ObjectTree().Intersect(shadow_ray, shadow_hit)) {
      return scalar(0.0);
    }
    return scalar(1.0);
  }
}

bool Tracer::TraceRay(
    const Ray& ray, TraceInfo& info, const unsigned depth) const {
  if (depth > m_scene->Config().max_recursions) {
    return false;
  }

  // Intersect scene.
  HitInfo hit = HitInfo::CreateNoHit();
  if (!m_scene->ObjectTree().Intersect(ray, hit)) {
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

  // Compensate material properties to preserve energy.
  vec3 reflection = material->Mirror() ? material_param.specular : vec3(0.0);
  vec3 transparency = material_param.transparency;
  vec3 diffuse = material_param.diffuse;
  transparency = transparency * (vec3(1.0) - reflection);
  diffuse = diffuse * (vec3(1.0) - transparency) * (vec3(1.0) - reflection);
  material_param.transparency = transparency;
  material_param.diffuse = diffuse;

  bool has_reflection = reflection.AbsSqr() > EPSILON;
  bool has_transparency = transparency.AbsSqr() > EPSILON;
  bool has_diffuse = diffuse.AbsSqr() > EPSILON;
  bool has_specular = material_param.specular.AbsSqr() > EPSILON;

  // Reflection?
  if (has_reflection) {
    // Reflected direction.
    vec3 reflect_dir = ray.Direction() -
        material_param.normal * (scalar(2.0) * material_param.normal.Dot(ray.Direction()));

    // Nudge origin point in order to avoid re-intersecting the origin surface.
    // TODO(mage): The "nudge distance" should be relative to object scale
    // somehow.
    vec3 reflect_start = hit.point + reflect_dir * scalar(0.0001);

    // Trace ray.
    Ray reflect_ray(reflect_start, reflect_dir);
    TraceInfo reflect_info;
    if (TraceRay(reflect_ray, reflect_info, depth + 1)) {
      info.color += reflect_info.color * reflection;
    }
  }

  // Transparency?
  if (has_transparency) {
    // Refracted direction.
    vec3 refract_dir = ray.Direction().Normalize().Refract(material_param.normal, material->Ior());

    // Nudge origin point in order to avoid re-intersecting the origin surface.
    // TODO(mage): The "nudge distance" should be relative to object scale
    // somehow.
    vec3 refract_start = hit.point + refract_dir * scalar(0.0001);

    // Trace ray.
    scalar filter = material_param.transparency.Abs();
    Ray refract_ray(refract_start, refract_dir);
    TraceInfo refract_info;
    if (TraceRay(refract_ray, refract_info, depth + 1)) {
      info.color += refract_info.color * transparency;
      filter *= scalar(1.0) - refract_info.alpha;
    }

    info.alpha = scalar(1.0) - filter;
  }

  // Light contribution.
  vec3 light_contrib(0);
  if (has_diffuse || has_specular) {
    // Iterate all the lights in the scene.
    for (auto it = m_scene->Lights().begin(); it != m_scene->Lights().end();
        it++) {
      Light* light = it->get();

      // Direction and distance to the light.
      vec3 light_dir = light->Position() - hit.point;
      scalar light_dist = light_dir.Abs();
      light_dir = light_dir * (scalar(1.0) / light_dist);

      scalar cos_alpha = light_dir.Dot(material_param.normal);
      if (cos_alpha >= scalar(0.0)) {
        Shader::LightParam light_param;
        light_param.light = light;
        light_param.dir = light_dir;
        light_param.dist = light_dist;
        light_param.cos_alpha = cos_alpha;
        light_param.amount = scalar(1.0);

        // Determine light visibility factor (0.0 for completely shadowed).
        light_param.amount = Shadow(light, hit.point);

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
