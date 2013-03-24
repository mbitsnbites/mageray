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

#include <atomic>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>

#include <tinyxml2.h>

#include "base/perf.h"

namespace {

vec3 ParseVec3String(const char* str) {
  // TODO(mage): Implement me!
  return vec3(0);
}

scalar ParseScalarString(const char* str) {
  // TODO(mage): Implement me!
  return 0.0;
}

}

class scene_parse_error : public std::exception {
  public:
    scene_parse_error(const tinyxml2::XMLNode* node, const char* msg) noexcept {
      m_msg = msg;
      const char* name = node->Value();
      m_name = name ? name : "?";
    }

    virtual const char* what() const noexcept {
      std::stringstream ss;
      ss << "In XML node " << m_name << ": " << m_msg;
      return ss.str().c_str();
    }

  private:
    std::string m_msg;
    std::string m_name;
};

void Scene::Reset() {
  m_camera.Reset();

  m_images.clear();
  m_meshes.clear();
  m_materials.clear();

  m_lights.clear();
  m_objects.clear();
}

void Scene::LoadCamera(tinyxml2::XMLElement* element) {
  if (const char* str = element->Attribute("position")) {
    m_camera.SetPosition(ParseVec3String(str));
  }
  if (const char* str = element->Attribute("lookat")) {
    m_camera.SetLookAt(ParseVec3String(str));
  }
  if (const char* str = element->Attribute("nominalup")) {
    m_camera.SetNominalUp(ParseVec3String(str));
  }
  if (const char* str = element->Attribute("fov")) {
    m_camera.SetFOV(ParseScalarString(str));
  }
}

void Scene::LoadImage(tinyxml2::XMLElement* element) {
  // TODO(mage): Implement me!
}

void Scene::LoadMesh(tinyxml2::XMLElement* element) {
  const char* name = element->Attribute("name");
  const char* file = element->Attribute("file");
  if (!name) {
    throw scene_parse_error(element, "Missing name attribute.");
  }
  if (!file) {
    throw scene_parse_error(element, "Missing file attribute.");
  }

  // Load the mesh.
  std::unique_ptr<Mesh> mesh(new Mesh());
  if (!mesh.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }
  std::string file_name = m_file_path + file;
  DLOG("Loading mesh file %s.", file_name.c_str());
  if (!mesh->Load(file_name.c_str())) {
    throw scene_parse_error(element, "Unable to load mesh file.");
  }

#ifdef _DEBUG
  std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
#endif

  m_meshes[name] = std::move(mesh);
}

void Scene::LoadMaterial(tinyxml2::XMLElement* element) {
  // TODO(mage): Implement me!
}

void Scene::LoadMeshObject(tinyxml2::XMLElement* element) {
  const char* mesh = element->Attribute("mesh");
  if (!mesh) {
    throw scene_parse_error(element, "Missing mesh attribute.");
  }

  std::unique_ptr<MeshObject> obj(new MeshObject());
  if (!obj.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }

  // Find the named mesh.
  std::map<std::string, std::unique_ptr<Mesh> >::iterator it = m_meshes.find(mesh);
  if (it == m_meshes.end()) {
    throw scene_parse_error(element, "Unable to find mesh (wrong name?).");
  }

  // Assign a mesh to the object.
  obj->SetMesh(it->second.get());

  // Collect transforms.
  // TODO(mage): Implement me!

  // Add the object to the object list.
  m_objects.push_back(std::move(obj));
}

void Scene::LoadLight(tinyxml2::XMLElement* element) {
  // TODO(mage): Implement me!
}

bool Scene::LoadFromXML(const char* file_name) {
  std::ifstream is(file_name, std::ios_base::in);
  if (is.good()) {
    return LoadFromXML(is);
  }
  LOG("Unable to open XML file %s", file_name);
  return false;
}

bool Scene::LoadFromXML(std::istream& stream) {
  Reset();

  bool success = true;
  try {
    // Load and parse XML file.
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err = doc.LoadFile(stream);
    if (err != tinyxml2::XML_NO_ERROR) {
      throw scene_parse_error(&doc, "Unable to load XML stream.");
    }

    // Load mageray node.
    tinyxml2::XMLElement* mageray_node = doc.FirstChildElement("mageray");
    if (!mageray_node) {
      throw scene_parse_error(&doc, "Missing mageray element.");
    }
    // TODO(mage): Check version number.

    // Load scene node.
    tinyxml2::XMLElement* scene_node = mageray_node->FirstChildElement("scene");
    if (!scene_node) {
      throw scene_parse_error(mageray_node, "Missing scene element.");
    }

    // Load the camera.
    if (tinyxml2::XMLElement* node = scene_node->FirstChildElement("camera")) {
      LoadCamera(node);
    }

    // Load images.
    if (tinyxml2::XMLElement* images_node = scene_node->FirstChildElement("images")) {
      // TODO(mage): Implement me!
    }

    // Load meshes.
    if (tinyxml2::XMLElement* meshes_node = scene_node->FirstChildElement("meshes")) {
      tinyxml2::XMLElement* node = meshes_node->FirstChildElement();
      while (node) {
        if (std::string(node->Value()) == "mesh") {
          LoadMesh(node);
        }
        node = node->NextSiblingElement();
      }
    }

    // Load materials.
    if (tinyxml2::XMLElement* materials_node = scene_node->FirstChildElement("materials")) {
      // TODO(mage): Implement me!
    }

    // Load objects.
    if (tinyxml2::XMLElement* objects_node = scene_node->FirstChildElement("objects")) {
      tinyxml2::XMLElement* node = objects_node->FirstChildElement();
      while (node) {
        if (std::string(node->Value()) == "meshobject") {
          LoadMeshObject(node);
        }
        node = node->NextSiblingElement();
      }
    }
  }
  catch (scene_parse_error err) {
    LOG("Error loading XML scene: %s", err.what());
    success = false;
  }

#if 0
  // In the mean time, we're just doing a default hard-coded scene...

  // Set up camera.
  m_camera.SetPosition(vec3(-3.0, -8.0, 8.0));
  m_camera.SetLookAt(vec3(0.0, 0.0, 3.0));

  // Load meshes.
#if 0
  {
    std::unique_ptr<Mesh> mesh(new Mesh());
    if (mesh.get() && mesh->Load("../resources/lucy.ctm")) {
      std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
      m_meshes["lucy"] = std::move(mesh);
    }
  }
#endif
  {
    std::unique_ptr<Mesh> mesh(new Mesh());
    if (mesh.get() && mesh->Load("../resources/bunny.ctm")) {
      std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
      m_meshes["bunny"] = std::move(mesh);
    }
  }
  {
    std::unique_ptr<Mesh> mesh(new Mesh());
    if (mesh.get() && mesh->Load("../resources/happy.ctm")) {
      std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
      m_meshes["happy"] = std::move(mesh);
    }
  }

  // Create objects.
  for (scalar x = -5.0; x <= 5.0; x += 2.5) {
    for (scalar y = -5.0; y <= 5.0; y += 2.5) {
      std::unique_ptr<MeshObject> obj(new MeshObject());
      if (obj.get()) {
        // Assign a mesh to the object.
        std::map<std::string, std::unique_ptr<Mesh> >::iterator it = m_meshes.find("happy");
        if (it != m_meshes.end()) {
          obj->SetMesh(it->second.get());
        }

        // Transform the object.
        obj->Translate(vec3(x, y, 0.0));

        // Add the object to the object list.
        m_objects.push_back(std::move(obj));
      }
    }
  }
  {
    std::unique_ptr<MeshObject> obj(new MeshObject());
    if (obj.get()) {
      // Assign a mesh to the object.
      std::map<std::string, std::unique_ptr<Mesh> >::iterator it = m_meshes.find("bunny");
      if (it != m_meshes.end()) {
        obj->SetMesh(it->second.get());
      }

      // Transform the object.
      obj->Translate(vec3(0.0, 0.0, 4.0));

      // Add the object to the object list.
      m_objects.push_back(std::move(obj));
    }
  }
#endif

  if (success) {
    // Build object tree.
    m_object_tree.Build(m_objects);
  }

  return success;
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

  std::atomic_uint hits(0), misses(0);

  // Loop over rows.
  #pragma omp parallel for
  for (int v = 0; v < image.Height(); ++v) {
    vec3 dir = forward + u_step * (-0.5 * width) +
      v_step * (static_cast<scalar>(v) - 0.5 * height);

    // Loop over columns in the row.
    for (int u = 0; u < image.Width(); ++u) {
      // Construct a ray.
      Ray ray(cam_pos, dir);

      // Shoot a ray against the object tree, containing all the objects in
      // the scene.
      HitInfo hit = HitInfo::CreateNoHit();
      Pixel result(0);
      if (m_object_tree.Intersect(ray, hit)) {
        scalar s = 1.0 - std::min(hit.t * 0.06, 1.0);
        result = Pixel(s, s, s);
        ++hits;
      } else {
        ++misses;
      }
      image.PixelAt(u, v) = result;

      dir += u_step;
    }
  }

  _raytrace.Done();

  std::cout << "hits=" << hits << " misses=" << misses << std::endl;
}
