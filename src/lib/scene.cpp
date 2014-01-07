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

#include <algorithm>
#include <atomic>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>

#include <tinyxml2.h>

#include "base/perf.h"
#include "base/platform.h"

namespace mageray {

class scene_parse_error : public std::exception {
  public:
    scene_parse_error(const tinyxml2::XMLNode* node, const char* msg) noexcept {
      m_msg = msg;
      const char* name = node ? node->Value() : NULL;
      m_name = name ? name : "?";
    }

    virtual ~scene_parse_error() noexcept {}

    virtual const char* what() const noexcept {
      return "A scene parser error occured.";
    }

    std::string message() const {
      std::ostringstream ss;
      ss << "In XML node " << m_name << ": " << m_msg;
      return ss.str();
    }

  private:
    std::string m_msg;
    std::string m_name;
};

void Scene::Reset() {
  m_images.clear();
  m_meshes.clear();
  m_shaders.clear();

  ClearDefinition();

  // Default configuration.
  m_config.max_recursions = 4;
  m_config.antialias_depth = 0;
  m_config.soft_shadow_depth = 3;
  m_config.max_photons = 0;
  m_config.max_photon_depth = 4;
  m_config.photon_energy = scalar(1000.0);
  m_config.direct_lighting = true;
  m_config.start_t = 0.0f;
  m_config.stop_t = 0.0f;
  m_config.num_frames = 1;

  InitDefaultShaders();
}

void Scene::SetTime(scalar t) {
  m_expression_parser.SetTime(t);
}

void Scene::ClearDefinition() {
  m_camera.Reset();
  m_materials.clear();
  m_lights.clear();
  m_objects.clear();
}

void Scene::LoadConfig(tinyxml2::XMLElement* element) {
  if (const char* str = element->Attribute("max_recursions")) {
    m_config.max_recursions = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("antialias_depth")) {
    m_config.antialias_depth = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("soft_shadow_depth")) {
    m_config.soft_shadow_depth = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("max_photons")) {
    m_config.max_photons = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("max_photon_depth")) {
    m_config.max_photon_depth = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("photon_energy")) {
    m_config.photon_energy = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("direct_lighting")) {
    m_config.direct_lighting = m_expression_parser.ToBool(str);
  }

  // Animation.
  if (const char* str = element->Attribute("start_t")) {
    m_config.start_t = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("stop_t")) {
    m_config.stop_t = m_expression_parser.ToScalar(str);
  }
  if (const char* str = element->Attribute("num_frames")) {
    m_config.num_frames = m_expression_parser.ToInt(str);
  }
}

void Scene::LoadAssets(tinyxml2::XMLElement* element) {
  // Load images.
  if (tinyxml2::XMLElement* images_node = element->FirstChildElement("images")) {
    tinyxml2::XMLElement* node = images_node->FirstChildElement();
    while (node) {
      if (std::string(node->Value()) == "image") {
        LoadImage(node);
      }
      node = node->NextSiblingElement();
    }
  }

  // Load meshes.
  if (tinyxml2::XMLElement* meshes_node = element->FirstChildElement("meshes")) {
    tinyxml2::XMLElement* node = meshes_node->FirstChildElement();
    while (node) {
      if (std::string(node->Value()) == "mesh") {
        LoadMesh(node);
      }
      else if (std::string(node->Value()) == "spheremesh") {
        LoadSphereMesh(node);
      }
      else if (std::string(node->Value()) == "planemesh") {
        LoadPlaneMesh(node);
      }
      node = node->NextSiblingElement();
    }
  }
}

void Scene::LoadCamera(tinyxml2::XMLElement* element) {
  if (const char* str = element->Attribute("position")) {
    m_camera.SetPosition(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("lookat")) {
    m_camera.SetLookAt(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("nominalup")) {
    m_camera.SetNominalUp(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("fov")) {
    m_camera.SetFOV(m_expression_parser.ToScalar(str));
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
  std::string file_name = m_file_path + file;
  DLOG("Loading mesh file %s.", file_name.c_str());
  std::unique_ptr<Mesh> mesh(Mesh::Load(file_name.c_str()));
  if (!mesh.get()) {
    std::string msg = std::string("Unable to load mesh file \"") +
        file_name + std::string("\".");
    throw scene_parse_error(element, msg.c_str());
  }

#ifdef _DEBUG
  std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
#endif

  m_meshes[name] = std::move(mesh);
}

void Scene::LoadSphereMesh(tinyxml2::XMLElement* element) {
  const char* name = element->Attribute("name");
  const char* radius_str = element->Attribute("radius");
  const char* resolution_str = element->Attribute("resolution");
  if (!name) {
    throw scene_parse_error(element, "Missing name attribute.");
  }
  if (!radius_str) {
    throw scene_parse_error(element, "Missing radius attribute.");
  }
  if (!resolution_str) {
    throw scene_parse_error(element, "Missing resolution attribute.");
  }
  scalar radius = m_expression_parser.ToScalar(radius_str);
  int resolution = m_expression_parser.ToInt(resolution_str);

  // Create a new sphere mesh.
  std::unique_ptr<Mesh> mesh(Mesh::MakeSphere(resolution, radius));
  if (!mesh.get()) {
    throw scene_parse_error(element, "Unable to create sphere mesh.");
  }

#ifdef _DEBUG
  std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
#endif

  m_meshes[name] = std::move(mesh);
}

void Scene::LoadPlaneMesh(tinyxml2::XMLElement* element) {
  const char* name = element->Attribute("name");
  const char* size_str = element->Attribute("size");
  if (!name) {
    throw scene_parse_error(element, "Missing name attribute.");
  }
  if (!size_str) {
    throw scene_parse_error(element, "Missing size attribute.");
  }
  vec2 size = m_expression_parser.ToVec2(size_str);

  // Create a new plane mesh.
  std::unique_ptr<Mesh> mesh(Mesh::MakePlane(size));
  if (!mesh.get()) {
    throw scene_parse_error(element, "Unable to create sphere mesh.");
  }

#ifdef _DEBUG
  std::cout << "Mesh bounding box: " << mesh->BoundingBox() << std::endl;
#endif

  m_meshes[name] = std::move(mesh);
}

void Scene::LoadMaterialSampler(tinyxml2::XMLElement* element, Sampler& sampler) {
  const char* image_name = element->Attribute("image");
  if (!image_name) {
    throw scene_parse_error(element, "Missing image attribute.");
  }

  // Set image for this sampler.
  auto it = m_images.find(image_name);
  if (it == m_images.end()) {
    std::string msg = std::string("Unable to find image \"") +
        image_name + std::string("\" (wrong name?).");
    throw scene_parse_error(element, msg.c_str());
  }
  sampler.SetImage(it->second.get());
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

  if (const char* str = element->Attribute("ambient")) {
    material->SetAmbient(m_expression_parser.ToScalar(str));
  }
  if (const char* str = element->Attribute("diffuse")) {
    material->SetDiffuse(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("specular")) {
    material->SetSpecular(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("transparency")) {
    material->SetTransparency(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("mirror")) {
    material->SetMirror(m_expression_parser.ToBool(str));
  }
  if (const char* str = element->Attribute("hardness")) {
    material->SetHardness(m_expression_parser.ToScalar(str));
  }
  if (const char* str = element->Attribute("ior")) {
    material->SetIor(m_expression_parser.ToScalar(str));
  }

  // Select shader (fall back to @phong if none given).
  const char* shader_name = element->Attribute("shader");
  if (!shader_name) {
    shader_name = "@phong";
  }

  // Set the shader for this material.
  auto it = m_shaders.find(shader_name);
  if (it == m_shaders.end()) {
    std::string msg = std::string("Unable to find shader \"") +
        shader_name + std::string("\" (wrong name?).");
    throw scene_parse_error(element, msg.c_str());
  }
  material->SetShader(it->second.get());

  // Get diffuse map (if any).
  if (tinyxml2::XMLElement* node = element->FirstChildElement("diffusemap")) {
    LoadMaterialSampler(node, material->DiffuseMap());
  }

  // Get specular map (if any).
  if (tinyxml2::XMLElement* node = element->FirstChildElement("specularmap")) {
    LoadMaterialSampler(node, material->SpecularMap());
  }

  // Get normal map (if any).
  if (tinyxml2::XMLElement* node = element->FirstChildElement("normalmap")) {
    LoadMaterialSampler(node, material->NormalMap());
  }

  m_materials[name] = std::move(material);
}

void Scene::LoadObject(tinyxml2::XMLElement* element) {
  const char* mesh = element->Attribute("mesh");
  if (!mesh) {
    throw scene_parse_error(element, "Missing mesh attribute.");
  }

  // Find the named mesh.
  auto it = m_meshes.find(mesh);
  if (it == m_meshes.end()) {
    std::string msg = std::string("Unable to find mesh \"") +
        mesh + std::string("\" (wrong name?).");
    throw scene_parse_error(element, msg.c_str());
  }

  // Create an object from the mesh.
  std::unique_ptr<Object> object(new Object(it->second.get()));

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
      object->Translate(m_expression_parser.ToVec3(node->GetText()));
    }

    // Scale transformation?
    else if (std::string(node->Value()) == "scale") {
      object->Scale(m_expression_parser.ToVec3(node->GetText()));
    }

    // Rotate transformation?
    else if (std::string(node->Value()) == "rotate") {
      object->Rotate(m_expression_parser.ToVec3(node->GetText()));
    }

    // Material
    else if (std::string(node->Value()) == "material") {
      // TODO(mage): Implement me!
    }

    node = node->NextSiblingElement();
  }

  // Add the object to the object list.
  m_objects.push_back(std::move(object));
}

void Scene::LoadLight(tinyxml2::XMLElement* element) {
  std::unique_ptr<Light> light(new Light());
  if (!light.get()) {
    throw scene_parse_error(element, "Out of memory?");
  }

  if (const char* str = element->Attribute("color")) {
    light->SetColor(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("position")) {
    light->SetPosition(m_expression_parser.ToVec3(str));
  }
  if (const char* str = element->Attribute("distance")) {
    light->SetDistance(m_expression_parser.ToScalar(str));
  }
  if (const char* str = element->Attribute("size")) {
    light->SetSize(m_expression_parser.ToScalar(str));
  }

  // Add the object to the object list.
  m_lights.push_back(std::move(light));
}

bool Scene::LoadAssets(const char* file_name) {
  std::ifstream is(file_name, std::ios_base::in);
  if (is.good()) {
    return LoadAssets(is);
  }
  LOG("Unable to open XML file %s", file_name);
  return false;
}

bool Scene::LoadAssets(std::istream& stream) {
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

    // Load config node (if any).
    if (tinyxml2::XMLElement* node = mageray_node->FirstChildElement("config")) {
      LoadConfig(node);
    }

    // Load assets node (if any).
    if (tinyxml2::XMLElement* node = mageray_node->FirstChildElement("assets")) {
      LoadAssets(node);
    }
  }
  catch (const scene_parse_error& err) {
    std::string msg = err.message();
    LOG("Error loading XML scene: %s", msg.c_str());
    success = false;
  }
  catch (const ExpressionParser::error& err) {
    std::string msg = err.message();
    LOG("Error loading XML scene: %s", msg.c_str());
    success = false;
  }
  catch (...) {
    LOG("Error loading XML scene.");
    success = false;
  }

  return success;
}

bool Scene::LoadDefinition(const char* file_name) {
  std::ifstream is(file_name, std::ios_base::in);
  if (is.good()) {
    return LoadDefinition(is);
  }
  LOG("Unable to open XML file %s", file_name);
  return false;
}

bool Scene::LoadDefinition(std::istream& stream) {
  ClearDefinition();

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

    // Load lights.
    if (tinyxml2::XMLElement* lights_node = scene_node->FirstChildElement("lights")) {
      tinyxml2::XMLElement* node = lights_node->FirstChildElement();
      while (node) {
        if (std::string(node->Value()) == "light") {
          LoadLight(node);
        }
        node = node->NextSiblingElement();
      }
    }

    // Load objects.
    if (tinyxml2::XMLElement* objects_node = scene_node->FirstChildElement("objects")) {
      tinyxml2::XMLElement* node = objects_node->FirstChildElement();
      while (node) {
        if (std::string(node->Value()) == "object") {
          LoadObject(node);
        }
        node = node->NextSiblingElement();
      }
    }
  }
  catch (const scene_parse_error& err) {
    std::string msg = err.message();
    LOG("Error loading XML scene: %s", msg.c_str());
    success = false;
  }
  catch (const ExpressionParser::error& err) {
    std::string msg = err.message();
    LOG("Error loading XML scene: %s", msg.c_str());
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

void Scene::InitDefaultShaders() {
  m_shaders["@null"].reset(new NullShader());
  m_shaders["@phong"].reset(new PhongShader());
}

} // namespace mageray
