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

class scene_parse_error : public std::exception {
  public:
    scene_parse_error(const tinyxml2::XMLNode* node, const char* msg) noexcept {
      m_msg = msg;
      const char* name = node ? node->Value() : NULL;
      m_name = name ? name : "?";
    }

    virtual ~scene_parse_error() noexcept {}

    virtual const char* what() const noexcept {
      std::ostringstream ss;
      ss << "In XML node " << m_name << ": " << m_msg;
      return ss.str().c_str();
    }

  private:
    std::string m_msg;
    std::string m_name;
};

namespace {

vec3 ParseVec3String(const char* str) {
  std::istringstream ss(str);
  vec3 v;
  ss >> v.x;
  ss >> v.y;
  ss >> v.z;
  if (ss.fail()) {
    std::string msg = std::string("Unable to parse \"") +
        str + std::string("\" as a vec3.");
    throw scene_parse_error(NULL, msg.c_str());
  }
  return v;
}

scalar ParseScalarString(const char* str) {
  std::istringstream ss(str);
  scalar s;
  ss >> s;
  if (ss.fail()) {
    std::string msg = std::string("Unable to parse \"") +
        str + std::string("\" as a scalar.");
    throw scene_parse_error(NULL, msg.c_str());
  }
  return s;
}

}

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
  const char* name = element->Attribute("name");
  const char* file = element->Attribute("file");
  if (!name) {
    throw scene_parse_error(element, "Missing name attribute.");
  }
  if (!file) {
    throw scene_parse_error(element, "Missing file attribute.");
  }

  // Load the image.
  std::unique_ptr<Image> image(new Image());
  if (!image.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }
  std::string file_name = m_file_path + file;
  DLOG("Loading image file %s.", file_name.c_str());
  if (!image->Load(file_name.c_str())) {
    std::string msg = std::string("Unable to load image file \"") +
        file_name + std::string("\".");
    throw scene_parse_error(element, msg.c_str());
  }

  m_images[name] = std::move(image);
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
    std::string msg = std::string("Unable to load mesh file \"") +
        file_name + std::string("\".");
    throw scene_parse_error(element, msg.c_str());
  }

#ifdef _DEBUG
  std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
#endif

  m_meshes[name] = std::move(mesh);
}

void Scene::LoadMaterial(tinyxml2::XMLElement* element) {
  const char* name = element->Attribute("name");
  if (!name) {
    throw scene_parse_error(element, "Missing name attribute.");
  }

  std::unique_ptr<Material> material(new Material());
  if (!material.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }

  // TODO(mage): Parse material properties.

  m_materials[name] = std::move(material);
}

void Scene::LoadObject(tinyxml2::XMLElement* element, Object* object) {
  // Get the objec material (if any)?
  const char* material = element->Attribute("material");
  if (material) {
    auto it = m_materials.find(material);
    if (it == m_materials.end()) {
      std::string msg = std::string("Unable to find material \"") +
          material + std::string("\" (wrong name?).");
      throw scene_parse_error(element, msg.c_str());
    }
    object->SetMaterial(it->second.get());
  }

  // Collect transformations.
  tinyxml2::XMLElement* node = element->FirstChildElement();
  while (node) {
    // Translate transformation?
    if (std::string(node->Value()) == "translate") {
      object->Translate(ParseVec3String(node->GetText()));
    }

    // Scale transformation?
    else if (std::string(node->Value()) == "scale") {
      object->Scale(ParseVec3String(node->GetText()));
    }

    // Rotate transformation?
    else if (std::string(node->Value()) == "rotate") {
      object->Rotate(ParseVec3String(node->GetText()));
    }

    // Material
    else if (std::string(node->Value()) == "material") {
      // TODO(mage): Implement me!
    }

    node = node->NextSiblingElement();
  }
}

void Scene::LoadMeshObject(tinyxml2::XMLElement* element) {
  const char* mesh = element->Attribute("mesh");
  if (!mesh) {
    throw scene_parse_error(element, "Missing mesh attribute.");
  }

  std::unique_ptr<MeshObject> object(new MeshObject());
  if (!object.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }

  // Find the named mesh.
  auto it = m_meshes.find(mesh);
  if (it == m_meshes.end()) {
    std::string msg = std::string("Unable to find mesh \"") +
        mesh + std::string("\" (wrong name?).");
    throw scene_parse_error(element, msg.c_str());
  }

  // Assign a mesh to the object.
  object->SetMesh(it->second.get());

  // Collect generic object information.
  LoadObject(element, object.get());

  // Add the object to the object list.
  m_objects.push_back(std::move(object));
}

void Scene::LoadSphereObject(tinyxml2::XMLElement* element) {
  const char* radius = element->Attribute("radius");
  if (!radius) {
    throw scene_parse_error(element, "Missing radius attribute.");
  }

  std::unique_ptr<SphereObject> object(new SphereObject());
  if (!object.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }

  // Set the radius for the sphere.
  object->SetRadius(ParseScalarString(radius));

  // Collect generic object information.
  LoadObject(element, object.get());

  // Add the object to the object list.
  m_objects.push_back(std::move(object));
}

void Scene::LoadLight(tinyxml2::XMLElement* element) {
  std::unique_ptr<Light> light(new Light());
  if (!light.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }

  // TODO(mage): Parse light properties.

  // Add the object to the object list.
  m_lights.push_back(std::move(light));
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
      tinyxml2::XMLElement* node = images_node->FirstChildElement();
      while (node) {
        if (std::string(node->Value()) == "image") {
          LoadImage(node);
        }
        node = node->NextSiblingElement();
      }
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
      tinyxml2::XMLElement* node = materials_node->FirstChildElement();
      while (node) {
        if (std::string(node->Value()) == "material") {
          LoadMaterial(node);
        }
        node = node->NextSiblingElement();
      }
    }

    // Load objects.
    if (tinyxml2::XMLElement* objects_node = scene_node->FirstChildElement("objects")) {
      tinyxml2::XMLElement* node = objects_node->FirstChildElement();
      while (node) {
        if (std::string(node->Value()) == "meshobject") {
          LoadMeshObject(node);
        }
        else if (std::string(node->Value()) == "sphereobject") {
          LoadSphereObject(node);
        }
        node = node->NextSiblingElement();
      }
    }
  }
  catch (scene_parse_error err) {
    LOG("Error loading XML scene: %s", err.what());
    success = false;
  }
  catch (...) {
    LOG("Error loading XML scene.");
    success = false;
  }

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
#ifdef _DEBUG
  std::cout << "Camera: pos=" << cam_pos << " forward=" << forward << " right=" << right << " up=" << up << std::endl;
#endif

  scalar width = static_cast<scalar>(image.Width());
  scalar height = static_cast<scalar>(image.Height());
  vec3 u_step = right * (1.0 / height);
  vec3 v_step = up * (-1.0 / height);
#ifdef _DEBUG
  std::cout << "Camera: x_step=" << u_step << " y_step=" << v_step << std::endl;
#endif

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

  DLOG("hits=%d misses=%d", int(hits), int(misses));
}
