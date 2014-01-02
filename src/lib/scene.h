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

#ifndef MAGERAY_SCENE_H_
#define MAGERAY_SCENE_H_

#include <istream>
#include <list>
#include <map>
#include <memory>
#include <string>

#include "base/types.h"
#include "camera.h"
#include "image.h"
#include "mesh.h"
#include "material.h"
#include "light.h"
#include "object.h"
#include "shader.h"
#include "tree.h"

namespace tinyxml2 {
class XMLElement;
}

namespace mageray {

/// Ray-tracing configuration parameters.
// TODO(mage): This should be part of the tracer, but we put it here since we
// parse the XML file within the scene (which is really not right).
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

  /// Photon energy scale factor.
  scalar photon_energy;

  /// Enable direct lighting during ray tracing pass.
  /// If direct lighting is not enabled, the photon map is used for the direct
  /// lighting.
  bool direct_lighting;
};


class Scene {
  public:
    Scene() {
      Reset();
    }
    ~Scene() {}

    void Reset();

    /// Set the file path for scene assets.
    /// @param path The file path.
    void SetFilePath(const char* path) {
      m_file_path = std::string(path);
    }

    /// Load a scene from a file.
    /// @param file_name The name of the file to load.
    /// @returns true if successful.
    bool LoadFromXML(const char* file_name);

    /// Load a scene from a stream.
    /// @param stream The stream to load.
    /// @returns true if successful.
    bool LoadFromXML(std::istream& stream);

    /// Get the camera for this scene.
    const mageray::Camera& Camera() const {
      return m_camera;
    }

    /// Get the lights in this scene.
    const std::list<std::unique_ptr<Light> >& Lights() const {
      return m_lights;
    }

    /// Get the object tree for this scene.
    const mageray::ObjectTree& ObjectTree() const {
      return m_object_tree;
    }

    /// Get the ray tracing configuration settings.
    const TraceConfig& Config() const {
      return m_config;
    }

  private:
    void LoadConfig(tinyxml2::XMLElement* element);
    void LoadCamera(tinyxml2::XMLElement* element);
    void LoadImage(tinyxml2::XMLElement* element);
    void LoadMesh(tinyxml2::XMLElement* element);
    void LoadMaterialSampler(tinyxml2::XMLElement* element, Sampler& sampler);
    void LoadMaterial(tinyxml2::XMLElement* element);
    void LoadObject(tinyxml2::XMLElement* element, Object* object);
    void LoadMeshObject(tinyxml2::XMLElement* element);
    void LoadSphereObject(tinyxml2::XMLElement* element);
    void LoadPlaneObject(tinyxml2::XMLElement* element);
    void LoadLight(tinyxml2::XMLElement* element);

    void InitDefaultShaders();

    std::string m_file_path;

    mageray::Camera m_camera;

    mageray::ObjectTree m_object_tree;

    std::map<std::string, std::unique_ptr<Image> > m_images;
    std::map<std::string, std::unique_ptr<Mesh> > m_meshes;
    std::map<std::string, std::unique_ptr<Mesh> > m_generated_meshes;
    std::map<std::string, std::unique_ptr<Shader> > m_shaders;
    std::map<std::string, std::unique_ptr<Material> > m_materials;

    std::list<std::unique_ptr<Light> > m_lights;
    std::list<std::unique_ptr<Object> > m_objects;

    TraceConfig m_config;

    FORBID_COPY(Scene);
};

} // namespace mageray

#endif // MAGERAY_SCENE_H_
