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

#include <iostream>
#include <string>

#include "scene.h"

using namespace mageray;

std::string ExtractBasePath(const std::string& file_name) {
  size_t separator_pos = file_name.find_last_of("/\\");
  if (separator_pos == std::string::npos) {
    return "";
  }
  return file_name.substr(0, separator_pos + 1);
}

int main(int argc, const char* argv[]) {
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " scene image" << std::endl;
    return 0;
  }

  std::cout << "mageray v0.1" << std::endl;

  // Get arguments.
  const std::string scene_file(argv[1]);
  const std::string image_file(argv[2]);

  // Determine resource folder.
  const std::string file_path = ExtractBasePath(scene_file);

  // Load scene.
  std::cout << std::endl << "[Loading scene]" << std::endl;
  Scene scene;
  scene.SetFilePath(file_path.c_str());
  if (!scene.LoadFromXML(scene_file.c_str())) {
    return 0;
  }

  // TODO(mage): Collect image quality parameters from command line and/or
  // the scene file.
  unsigned width = 1024;
  unsigned height = 768;

  // Create a target image.
  Image img;
  if (!img.Allocate(width, height)) {
    std::cerr << "Unable to create target image (out of memory?)." << std::endl;
    return 0;
  }

  // Ray trace image.
  std::cout << std::endl << "[Rendering image]" << std::endl;
  scene.GenerateImage(img);

  // Save image.
  img.SavePNG(image_file.c_str());

  return 0;
}
