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
#include <vector>
#include <cmath>

#include "base/perf.h"
#include "camera.h"
#include "hitinfo.h"
#include "vec.h"
#include "mat.h"
#include "image.h"
#include "mesh.h"
#include "scene.h"
#include "triangle.h"

void TestVectors() {
  std::cout << "--- Vectors ---" << std::endl;

  vec3 x(1.0f, 0.0f, 1.0f), y;

  y = vec3(0.3f, 0.4f, 0.5f);

  x = x + y;
  vec3 z = y - x;
  vec3 g(2,3,4);
  vec3 h;
  h = g + vec3(0.01, 2.1, -0.3);
  h = h + vec3(1);

  std::cout << "x=" << x << ", z=" << z << " |g|=" << g.Abs() << " |h|=" << h.Abs() << std::endl;

  double d = x.Dot(g);
  vec3 v = x.Cross(vec3(1,0,0));

  std::cout << "d=" << d << " v=" << v << std::endl;

  vec3 n = vec3(1.0f,0.0f,0.0f).Cross(vec3(0.0f,1.0f,0.0f));
  n += vec3(0.0f);

  std::cout << "(1,0,0) x (0,1,0) = " << n << std::endl;
}

void TestMatrices() {
  std::cout << std::endl << "--- Matrices ---" << std::endl;

  mat3x4 m = mat3x4::Scale(2.0f);
  std::cout << "Scale matrix = " << m << std::endl;
  vec3 v = vec3(1.5f, 0.7f, -3.3f);
  std::cout << "Original vector = " << v << std::endl;
  std::cout << "Transformed vector = " << (m * v) << std::endl;
  m = mat3x4::Translate(5.1f, 5.2f, 4.3f) * m;
  std::cout << "Transformed matrix = " << m << std::endl;
  std::cout << "Transformed vector = " << (m * v) << std::endl;

  m = mat3x4( 1, 2, 3, 0,
             -1, 9, 2, 2,
              1, 3, 3, 5);
  std::cout << "Original matrix = " << m << std::endl;
  ScopedPerf _inv = ScopedPerf("Matrix inverse");
  mat3x4 mi = m.Inverse();
  _inv.Done();
  std::cout << "Inverse matrix = " << mi << std::endl;
  std::cout << "Original * inverse matrix = " << (m * mi) << std::endl;
}

void TestImages() {
  std::cout << std::endl << "--- Images ---" << std::endl;

  Image img1;
  ScopedPerf _load = ScopedPerf("Load image");
  if (img1.Load("../resources/autumn.png")) {
    _load.Done();
    std::cout << "Source image: " << img1.Width() << " x " << img1.Height()
        << " First pixel = " << img1.PixelAt(0,0) << "." << std::endl;

    Image img2;
    if (img2.Allocate(1920, 1080)) {
      std::cout << "Destination image: " << img2.Width() << " x "
          << img2.Height() << std::endl;

      std::cout << "Interpolating..." << std::endl;
      float s_step = 1.0f / static_cast<float>(img2.Width());
      float t_step = 1.0f / static_cast<float>(img2.Height());
      float t = 0.0f;
      ScopedPerf _interp = ScopedPerf("Interpolate image");
      for (int y = 0; y < img2.Height(); ++y) {
        float s = 0.0f;
        for (int x = 0; x < img2.Width(); ++x) {
          img2.PixelAt(x, y) = img1.Sample(s, t);
          s += s_step;
        }
        t += t_step;
      }
      _interp.Done();

      std::cout << "Saving..." << std::endl;
      ScopedPerf _save = ScopedPerf("Save image");
      img2.SavePNG("img_interpolated.png");
      _save.Done();
    }
  }
}

void TestScene() {
  std::cout << std::endl << "--- Scene ---" << std::endl;

  Scene scene;
  scene.SetFilePath("../resources/");
  if (scene.LoadFromXML("../resources/test.xml")) {
    // Create a target image.
    Image img;
    if (img.Allocate(1024, 768)) {
      // Ray trace image.
      scene.GenerateImage(img);

      // Save image.
      ScopedPerf _save = ScopedPerf("Save image");
      img.SavePNG("raytraced.png");
      _save.Done();
    }
  }
}

int main() {
  // --- Vectors ---
  TestVectors();

  // --- Matrices ---
  TestMatrices();

  // --- Images ---
  TestImages();

  // --- Scene ---
  TestScene();

  return 0;
}
