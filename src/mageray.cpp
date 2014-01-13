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

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include "async_image_saver.h"
#include "scene.h"
#include "tracer.h"

using namespace mageray;

namespace {

std::string ExtractBasePath(const std::string& file_name) {
  size_t separator_pos = file_name.find_last_of("/\\");
  if (separator_pos == std::string::npos) {
    return "";
  }
  return file_name.substr(0, separator_pos + 1);
}

std::string MakeAnimName(const std::string& file_name, unsigned frame) {
  std::string base, ext;
  size_t ext_pos = file_name.find_last_of(".");
  if (ext_pos == std::string::npos) {
    base = file_name;
    ext = ".png";
  }
  else {
    base = file_name.substr(0, ext_pos);
    ext = file_name.substr(ext_pos);
  }
  std::stringstream ss;
  std::ios_base::fmtflags flags = ss.flags();
  ss << base << "_";
  ss << std::setfill('0') << std::setw(6) << frame;
  ss.flags(flags);
  ss << ext;
  return ss.str();
}

void ShowUsage(const char* prg_name) {
  std::cout << "Usage: " << prg_name << " [options] scene image" << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << " -w width" << std::endl;
  std::cout << " -h height" << std::endl;
  std::cout << " -i iterations (for profiling)" << std::endl;
}

}

int main(int argc, const char* argv[]) {
  std::cout << "mageray v0.1" << std::endl;

  if (argc < 3) {
    ShowUsage(argv[0]);
    return 0;
  }

  // Collect command line arguments.
  unsigned width = 1024;
  unsigned height = 768;
  unsigned iterations = 1;
  const char* file_names[2];
  int file_name_idx = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "-w") {
      if ((i + 1) == argc) {
        std::cerr << "Too few arguments." << std::endl;
        ShowUsage(argv[0]);
        return 0;
      }
      ++i;
      width = std::stoi(argv[i]);
    }
    else if (a == "-h") {
      if ((i + 1) == argc) {
        std::cerr << "Too few arguments." << std::endl;
        ShowUsage(argv[0]);
        return 0;
      }
      ++i;
      height = std::stoi(argv[i]);
    }
    else if (a == "-i") {
      if ((i + 1) == argc) {
        std::cerr << "Too few arguments." << std::endl;
        ShowUsage(argv[0]);
        return 0;
      }
      ++i;
      iterations = std::stoi(argv[i]);
    }
    else {
      if (file_name_idx >= 2) {
        std::cerr << "Too many arguments." << std::endl;
        ShowUsage(argv[0]);
        return 0;
      }
      file_names[file_name_idx++] = argv[i];
    }
  }

  // Get file name arguments.
  const std::string scene_file(file_names[0]);
  const std::string image_file(file_names[1]);

  // Determine resource folder.
  const std::string file_path = ExtractBasePath(scene_file);

  // Load scene assets.
  std::cout << std::endl << "[Loading scene assets]" << std::endl;
  Scene scene;
  scene.SetFilePath(file_path.c_str());
  if (!scene.LoadAssets(scene_file.c_str())) {
    return 0;
  }
  Tracer tracer;
  tracer.SetScene(&scene);

  // Start the asynchronous image saver.
  AsyncImageSaver img_saver;

  // Animation configuration: time per frame.
  unsigned num_frames = std::max(scene.Config().num_frames, unsigned(1));
  scalar time_per_frame = (scene.Config().stop_t - scene.Config().start_t)
      / scalar(std::max(num_frames - 1, unsigned(1)));

  // Animation loop.
  for (unsigned frame = 1; frame <= scene.Config().num_frames; ++frame) {
    // Calculate current animation time.
    scene.SetTime(scene.Config().start_t + (frame - 1) * time_per_frame);

    // Load scene definition.
    std::cout << std::endl << "[Loading scene definition]" << std::endl;
    if (!scene.LoadDefinition(scene_file.c_str())) {
      return 0;
    }

    // Create a target image.
    std::unique_ptr<Image> img(new Image);
    img->Allocate(width, height);

    // Ray trace image.
    std::cout << std::endl << "[Rendering image (" << width << "x" << height << ")]" << std::endl;
    for (unsigned i = 1; i <= iterations; ++i) {
      if (iterations > 1) {
        std::cout << "Render iteration " << i << "/" << iterations << std::endl;
      }
      tracer.GenerateImage(img.get());
    }

    // Save image.
    std::string image_file_anim;
    if (scene.Config().num_frames > 1) {
      image_file_anim = MakeAnimName(image_file, frame);
    }
    else {
      image_file_anim = image_file;
    }
    img_saver.SavePNG(std::move(img), image_file_anim);
  }

  return 0;
}
