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
#include "tree.h"

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

    /// Generate an image of the current scene.
    /// @param image The image to render to.
    void GenerateImage(Image& image);

  private:
    std::string m_file_path;

    Camera m_camera;

    ObjectTree m_object_tree;

    std::map<std::string, std::unique_ptr<Image> > m_images;
    std::map<std::string, std::unique_ptr<Mesh> > m_meshes;
    std::map<std::string, std::unique_ptr<Material> > m_materials;

    std::list<std::unique_ptr<Light> > m_lights;
    std::list<std::unique_ptr<Object> > m_objects;

    FORBID_COPY(Scene);
};

#endif // MAGERAY_SCENE_H_
