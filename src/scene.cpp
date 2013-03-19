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

#include "scene.h"

#include <fstream>
#include <iostream>
#include <tinyxml2.h>

#include "base/perf.h"

using namespace tinyxml2;

void Scene::Reset() {
  m_file_path = std::string("");

  m_camera.Reset();
  m_images.clear();
  m_meshes.clear();
  m_materials.clear();
  m_lights.clear();
  m_objects.clear();
}

bool Scene::LoadFromXML(const char* file_name) {
  std::ifstream is(file_name, std::ios_base::in);
  if (is.good()) {
    return LoadFromXML(is);
  }
  return false;
}

bool Scene::LoadFromXML(std::istream& stream) {
  // Load and parse XML file.
  XMLDocument doc;
  if (doc.LoadFile(stream) != XML_NO_ERROR) {
    return false;
  }

  Reset();

  // TODO(mage): Implement me!

  // In the mean time, we're just doing a default dummy-scene...
  m_camera.SetPosition(vec3(1.0, 3.5, 0.0));
  m_camera.SetLookAt(vec3(2.0, -0.4, 1.0));

  std::unique_ptr<Mesh> mesh(new Mesh());
  if (mesh.get() && mesh->Load("../resources/lucy.ctm")) {
    m_meshes.push_back(std::move(mesh));
  }

  return true;
}

void Scene::GenerateImage(Image& image) {
  // Set up camera.
  vec3 cam_pos = m_camera.Matrix().TransformPoint(vec3(0));
  vec3 forward = m_camera.Matrix().TransformDirection(vec3(0,1,0));
  vec3 right = m_camera.Matrix().TransformDirection(vec3(1,0,0));
  vec3 up = m_camera.Matrix().TransformDirection(vec3(0,0,1));
  std::cout << "Camera: pos=" << cam_pos << " forward=" << forward << " right=" << right << " up=" << up << std::endl;

  scalar width = static_cast<scalar>(image.Width());
  scalar height = static_cast<scalar>(image.Height());
  vec3 u_step = right * (1.0 / height);
  vec3 v_step = up * (-1.0 / height);
  std::cout << "Camera: x_step=" << u_step << " y_step=" << v_step << std::endl;

  ScopedPerf _raytrace = ScopedPerf("Raytrace image");

  int hits = 0, misses = 0;

  // Loop over rows.
  #pragma omp parallel for
  for (int v = 0; v < image.Height(); ++v) {
    vec3 dir = forward + u_step * (-0.5 * width) +
      v_step * (static_cast<scalar>(v) - 0.5 * height);

    // Loop over columns in the row.
    for (int u = 0; u < image.Width(); ++u) {
      // Construct a ray.
      Ray ray(cam_pos, dir);

      // Shoot a ray against all the meshes in the scene.
      // TODO(mage): Should be against an object tree...
      HitInfo hit = HitInfo::CreateNoHit();
      std::list<std::unique_ptr<Mesh> >::iterator it;
      for (it = m_meshes.begin(); it != m_meshes.end(); it++) {
        Mesh* mesh = (*it).get();
        if (mesh->Intersect(ray, hit)) {
          scalar s = 1.0 - std::min(hit.t * 0.1, 1.0);
          image.PixelAt(u, v) = Pixel(s, s, s);
          ++hits;
        } else {
          image.PixelAt(u, v) = Pixel(0);
          ++misses;
        }
      }

      dir += u_step;
    }
  }

  _raytrace.Done();

  std::cout << "hits=" << hits << " misses=" << misses << std::endl;
}
