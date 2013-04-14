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

#include <cstdlib>
#include <list>
#include <thread>

#include "base/log.h"
#include "base/perf.h"
#include "base/threads.h"
#include "hitinfo.h"
#include "light.h"
#include "material.h"
#include "pixel.h"
#include "scene.h"
#include "shader.h"

namespace mageray {

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

Tracer::Tracer() : m_scene(NULL) {
  // Default configuration.
  m_config.max_recursions = 4;
  m_config.antialias_depth = 0;
  m_config.soft_shadow_depth = 3;
  m_config.max_photons = 1000000;
  m_config.max_photon_depth = 6;
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
  int left, top, width, height;
  while (controller->NextArea(left, top, width, height)) {
    // Loop over rows.
    int v0 = top;
    for (int  j = 0; j < height; ++j) {
      int v = v0 + j;

      // Every second row we do right to left.
      int u0 = j & 1 ? left + width - 1 : left;

      vec3 dir = forward +
          u_step * (static_cast<scalar>(u0) - scalar(0.5) * img_width) +
          v_step * (static_cast<scalar>(v) - scalar(0.5) * img_height);

      // Loop over columns in the row.
      for (int  i = 0; i < width; ++i) {
        int u = j & 1 ? u0 - i : u0 + i;

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

        if (j & 1) {
          dir -= u_step;
        } else {
          dir += u_step;
        }
      }
    }
  }
}

void Tracer::GeneratePhotonMap() {
  ASSERT(m_scene, "The scene is undefined.");
  ScopedPerf _photon_map = ScopedPerf("Generate photon map");

  // Allocate memory for the photons.
  m_photon_map.SetCapacity(m_config.max_photons);

  // Set up a random-access vector for all the light sources.
  std::vector<Light*> lights;
  lights.reserve(m_scene->m_lights.size());
  for (auto it = m_scene->m_lights.begin(); it != m_scene->m_lights.end();
      it++) {
    lights.push_back(it->get());
  }

  // Initialize a random number generator.
  Random rnd;

  // Generate photon map.
  std::atomic_int num_actual_rays(0);
  unsigned rays_per_light = 100;
  unsigned total_rays = (m_config.max_photons * 500) / rays_per_light;
  for (unsigned i = total_rays; i; --i) {
    // Select a random light source.
    Light* light = lights[rnd.Int(0, lights.size() - 1)];

    // Do several rays for this light source (better cache performance).
    for (unsigned j = rays_per_light; j; --j) {
      // Select a random direction.
      vec3 dir = rnd.SignedVec3().Normalize();

      // Shoot a ray into the scene.
      Ray ray(light->Position(), dir);
      TracePhoton(ray, rnd, 1, light->Color());
      num_actual_rays++;
    }

    if (m_photon_map.IsFull()) {
      break;
    }
  }

  // Adjust photon energy scale based on number of emitted light rays.
  int num_rays = num_actual_rays.load();
  m_photon_scale = scalar(19500.0) / static_cast<scalar>(num_rays);
  DLOG("Number of light rays shot: %d", num_rays);
  DLOG("Photon intensity scale: %f", double(m_photon_scale));

  // Build the KD tree.
  m_photon_map.BuildKDTree();

  _photon_map.Done();
}

void Tracer::GenerateImage(Image& image) const {
  ASSERT(m_scene, "The scene is undefined.");
  ScopedPerf _raytrace = ScopedPerf("Raytrace image");

  // Get level of hardware concurrency.
  int concurrency = Thread::hardware_concurrency();
  DLOG("Using %d threads to render image.", concurrency);

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

scalar Tracer::Shadow(const Light* light, const vec3& position) const {
  if (light->Size() > EPSILON && m_config.soft_shadow_depth > 0) {
    // Number of rays along each side of the light square.
    int count = (1 << m_config.soft_shadow_depth) + 1;

    // Compute light contribution using soft shadows.
    SoftShadower soft_shadower(m_scene->m_object_tree, light, position, count);
    return soft_shadower.TotalUnblocked();
  } else {
    // No soft shadow, just use a single ray.
    vec3 light_vec = position - light->Position();
    HitInfo shadow_hit = HitInfo::CreateShadowTest(scalar(0.9999));
    Ray shadow_ray(light->Position(), light_vec);
    if (m_scene->m_object_tree.Intersect(shadow_ray, shadow_hit)) {
      return scalar(0.0);
    }
    return scalar(1.0);
  }
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
#if 1
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

      scalar cos_alpha = light_dir.Dot(hit.normal);
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
#endif

  // Get light contribtion from photon map.
  if (m_photon_map.HasPhotons()) {
    unsigned max_photons = 100;
    scalar max_r = std::sqrt(scalar(max_photons) / PI) *
        m_photon_map.MedianDistance();
    vec3 photon_color = m_photon_map.GetTotalLightInRange(hit.point, hit.normal,
        max_r);
    light_contrib += material_param.diffuse * photon_color.Sqrt() * (m_photon_scale / (max_r * max_r));
  }

  // Run final shader pass.
  info.color += shader->FinalPass(surface_param, material_param, light_contrib);

  return true;
}

void Tracer::TracePhoton(const Ray& ray, Random& random,
    const unsigned depth, const vec3& color) {
  if (depth > m_config.max_photon_depth) {
    return;
  }

  // Intersect scene.
  HitInfo hit = HitInfo::CreateNoHit();
  if (!m_scene->m_object_tree.Intersect(ray, hit)) {
    return;
  }

  // Get surface properties.
  hit.object->CompleteHitInfo(ray, hit);

  // Get material.
  const Material* material = hit.object->Material();

  // Uh, nothing much to do here...
  if (UNLIKELY(!material || !material->Shader())) {
    return;
  }

  // View direction.
  vec3 view_dir = (hit.point - ray.Origin()).Normalize();
//  vec3 view_dir = ray.Direction().Normalize();

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

  vec3 mirror = material_param.specular * material->Mirror();
  scalar opacity = scalar(1.0) - material_param.alpha;

  // Monte-carlo, select which direction to take...
  scalar p_mirror = mirror.Abs();
  scalar p_transparency = opacity;
  scalar p_diffuse = material_param.diffuse.Abs();
  scalar p_total = p_mirror + p_transparency + p_diffuse;
  scalar selection = p_total * random.Scalar();

  // We're not interested in direct light (will be handled by direct rendering
  // pass).
#if 1
  if (depth > 1 && p_diffuse > scalar(0.0)) {
#else
  if (p_diffuse > scalar(0.0)) {
#endif
    // Produce a photon.
    Photon* photon = m_photon_map.NextPhoton();
    if (UNLIKELY(!photon)) {
      // TODO(mage): Have a way of reporting that the photon map is full...
      return;
    }

    // Fill out photon information.
    photon->position = hit.point;
    photon->direction = view_dir;
    photon->color = color;
  }

  // Reflection?
  if (selection < p_mirror) {
    // Reflected direction.
    vec3 reflect_dir = ray.Direction() -
        hit.normal * (scalar(2.0) * hit.normal.Dot(ray.Direction()));

    // Nudge origin point in order to avoid re-intersecting the origin surface.
    // TODO(mage): The "nudge distance" should be relative to object scale
    // somehow.
    vec3 reflect_start = hit.point + reflect_dir * scalar(0.0001);

    // Trace photon.
    Ray reflect_ray(reflect_start, reflect_dir);
    TracePhoton(reflect_ray, random, depth + 1,
        color * material_param.specular);
    return;
  }
  selection -= p_mirror;

  // Transparency?
  if (selection < p_transparency) {
    // Refracted direction.
    // TODO(mage): Implement me!
    vec3 refract_dir = ray.Direction();

    // Nudge origin point in order to avoid re-intersecting the origin surface.
    // TODO(mage): The "nudge distance" should be relative to object scale
    // somehow.
    vec3 refract_start = hit.point + refract_dir * scalar(0.0001);

    // Trace photon.
    Ray refract_ray(refract_start, refract_dir);
    TracePhoton(refract_ray, random, depth + 1, color);
    return;
  }
  selection -= p_transparency;

  // Diffuse reflection?
  vec3 diffuse_dir = (hit.normal + random.SignedVec3() * scalar(0.5)).Normalize();
  scalar cos_alpha = hit.normal.Dot(diffuse_dir);
  if (selection < cos_alpha * p_diffuse) {
    vec3 diffuse_start = hit.point + diffuse_dir * scalar(0.0001);
    Ray diffuse_ray(diffuse_start, diffuse_dir);
    TracePhoton(diffuse_ray, random, depth + 1,
        color * material_param.diffuse);
    return;
  }
}

} // namespace mageray
