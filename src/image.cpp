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

#include "image.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <vector>
#include <memory>

#include <png.h>

#include "base/log.h"

namespace {

template <int len>
bool MatchingSignature(std::istream& stream, const unsigned char* signature) {
  // Read len bytes from the stream.
  unsigned char buf[len];
  std::fill(buf, buf + len, 0);
  std::streampos old_pos = stream.tellg();
  stream.read(reinterpret_cast<char*>(buf), len);
  stream.seekg(old_pos);

  // Compate signature to the read buffer.
  return std::equal(buf, buf + len, signature);
}

void MyPNGRead(png_structp png_ptr, png_bytep data, png_size_t len) {
  std::istream* stream = static_cast<std::istream*>(png_get_io_ptr(png_ptr));
  ASSERT(stream, "No input stream found.");
  stream->read(reinterpret_cast<char*>(data), len);
}

void MyPNGWrite(png_structp png_ptr, png_bytep data, png_size_t len) {
  std::ostream* stream = static_cast<std::ostream*>(png_get_io_ptr(png_ptr));
  ASSERT(stream, "No output stream found.");
  stream->write(reinterpret_cast<char*>(data), len);
}

void MyPNGFlush(png_structp png_ptr) {
  std::ostream* stream = static_cast<std::ostream*>(png_get_io_ptr(png_ptr));
  ASSERT(stream, "No output stream found.");
  stream->flush();
}

}

void Image::Reset() {
  m_data.reset(NULL);
  m_width = 0;
  m_height = 0;
  m_s_scale = 0.0f;
  m_t_scale = 0.0f;
}

bool Image::Allocate(int width, int height) {
  ASSERT(width > 0 && height > 0,
      "Bad dimensions (%d x %d).", width, height);

  m_data.reset(new Pixel[width * height]);
  if (m_data.get() == NULL) {
    Reset();
    return false;
  }
  m_width = width;
  m_height = height;

  // Scaling factors for converting normalized s/t coordinates to fixed point
  // indices (used in the Sample() method).
  m_s_scale = 256.0f * static_cast<float>(m_width);
  m_t_scale = 256.0f * static_cast<float>(m_height);

  return true;
}

bool Image::LoadPNG(std::istream& stream) {
  static const unsigned char signature[] = {137, 80, 78, 71, 13, 10, 26, 10};
  if (!MatchingSignature<sizeof(signature)>(stream, signature)) {
    return false;
  }

  DLOG("Identified PNG file format.");

  // Initialize PNG data structures.
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
      NULL, NULL);
  if (!png_ptr) {
    return false;
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
    return false;
  }

  // Set up file interface.
  png_set_read_fn(png_ptr, &stream, MyPNGRead);

  // Get image information.
  png_read_info(png_ptr, info_ptr);
  int width = png_get_image_width(png_ptr, info_ptr);
  int height = png_get_image_height(png_ptr, info_ptr);
  int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
  int color_type = png_get_color_type(png_ptr, info_ptr);

  DLOG("PNG info: %d x %d, %d bits (color type %d).", width, height, bit_depth,
      color_type);

  // Set up transformations (we want 32-bit RGBA).
  if (color_type == PNG_COLOR_TYPE_PALETTE ||
      png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
    png_set_expand(png_ptr);
  }
  if (color_type == PNG_COLOR_TYPE_RGB ||
      color_type == PNG_COLOR_TYPE_GRAY) {
    png_set_add_alpha(png_ptr, 255, PNG_FILLER_AFTER);
  }
  if (color_type == PNG_COLOR_TYPE_GRAY ||
      color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
    png_set_gray_to_rgb(png_ptr);
  }
  if (bit_depth == 16) {
#if PNG_LIBPNG_VER >= 10504
     png_set_scale_16(png_ptr);
#else
     png_set_strip_16(png_ptr);
#endif
  }
  if (color_type == PNG_COLOR_TYPE_RGB ||
      color_type == PNG_COLOR_TYPE_RGB_ALPHA) {
    // TODO(mage): Only for little endian machines!
    png_set_bgr(png_ptr);
  }

  // Allocate memory for the image data.
  bool success = false;
  if (Allocate(width, height)) {
    // Set up row pointers.
    std::unique_ptr<png_bytep> row_pointers(new png_bytep[height]);
    for (int i = 0; i < height; ++i) {
      row_pointers.get()[i] = reinterpret_cast<png_bytep>(&PixelAt(0, i));
    }

    // Read the PNG file to the image data buffer.
    png_read_image(png_ptr, row_pointers.get());

    success = true;
  }

  // Free up resources.
  png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);

  return success;
}

bool Image::LoadJPEG(std::istream& stream) {
  static const unsigned char signature[] = {255, 216};
  if (!MatchingSignature<sizeof(signature)>(stream, signature)) {
    return false;
  }

  DLOG("Identified JPEG file format.");

  // TODO(mage): Implement me!
  LOG("JPEG reading support not yet implemented.");
  return false;
}


bool Image::Load(std::istream& stream) {
  // Try different file formats.
  if (LoadPNG(stream)) {
    return true;
  } else if (LoadJPEG(stream)) {
    return true;
  }

  LOG("Unable to load image file.");
  Reset();
  return false;
}

bool Image::Load(const char* file_name) {
  std::ifstream is(file_name, std::ios_base::in | std::ios_base::binary);
  if (is.good()) {
    return Load(is);
  }

  LOG("Unable to open image file %s.", file_name);
  Reset();
  return false;
}

bool Image::SavePNG(std::ostream& stream) {
  ASSERT(!Empty(), "No image data to write.");

  // Initialize PNG data structures.
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL,
      NULL, NULL);
  if (!png_ptr) {
    return false;
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
    return false;
  }

  // Set up file interface.
  png_set_write_fn(png_ptr, &stream, MyPNGWrite, MyPNGFlush);

  // Set up image information.
  png_set_IHDR(png_ptr,
               info_ptr,
               m_width,
               m_height,
               8,
               PNG_COLOR_TYPE_RGB_ALPHA,
               PNG_INTERLACE_NONE,
               PNG_COMPRESSION_TYPE_DEFAULT,
               PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png_ptr, info_ptr);

  // Transformations.
  // TODO(mage): Only for little endian machines!
  png_set_bgr(png_ptr);

  // Set up row pointers.
  std::unique_ptr<png_bytep> row_pointers(new png_bytep[m_height]);
  for (int i = 0; i < m_height; ++i) {
    row_pointers.get()[i] = reinterpret_cast<png_bytep>(&PixelAt(0, i));
  }

  // Write the PNG file to the image data buffer.
  png_write_image(png_ptr, row_pointers.get());

  // Free up resources.
  png_destroy_write_struct(&png_ptr, &info_ptr);

  return true;
}

bool Image::SavePNG(const char* file_name) {
  std::ofstream os(file_name, std::ios_base::out | std::ios_base::binary);
  if (os.good()) {
    return SavePNG(os);
  }
  return false;
}

void Image::Clear(const Pixel& color) {
  Pixel* ptr = m_data.get();
  if (!ptr) {
    return;
  }

  // Can we use memset (e.g. for black or white)? It's generally faster than
  // std::fill or a plain loop, since it's usually assembler optimized.
  int c = color.a();
  if (color.r() == c && color.g() == c && color.b() == c) {
    std::memset(ptr, c, sizeof(Pixel) * m_width * m_height);
    return;
  }

  // Use std::fill for setting all pixels to the same color.
  std::fill(ptr, ptr + m_width * m_height, color);
}

// This is a fairly quick version of linear interpolation between two 32-bit
// colors using an 8-bit fractional weight (0-255). It uses semi-packed
// multiplication (two color components per multiplication), which means that
// only two integer multiplications are used in the operation.
static inline Pixel::Composite Lerp(const Pixel::Composite c1,
    const Pixel::Composite c2, const int w) {
  const Pixel::Composite c1_a = c1 & 0x00ff00ff;
  const Pixel::Composite c1_b = c1 & 0xff00ff00;
  const Pixel::Composite c2_a = c2 & 0x00ff00ff;
  const Pixel::Composite c2_b = c2 & 0xff00ff00;
  return ((((c1_a << 8) + (c2_a - c1_a) * w) >> 8) & 0x00ff00ff) |
      (((c1_b + ((c2_b >> 8) - (c1_b >> 8)) * w)) & 0xff00ff00);
}

Pixel Image::Sample(float s, float t) const {
  // NOTE: Any kind of clamping or repeating has to be done before calling this
  // method.
  ASSERT(s >= 0.0f && s < 1.0f && t >= 0.0f && t < 1.0f,
      "Illegal coordinates (%f,%f).", s, t);

  // Convert floating point coordinates to fixed point coordinates (8-bit
  // sub-pixel precision).
  int s_fixed = static_cast<int>(s * m_s_scale);
  int t_fixed = static_cast<int>(t * m_t_scale);
  int si = s_fixed >> 8;
  int ti = t_fixed >> 8;
  int sw = s_fixed & 0xff;
  int tw = t_fixed & 0xff;

  ASSERT(si >= 0 && si < m_width && ti >= 0 && ti < m_height,
      "Indices out of range.");

  // Look up 4 neighbouring pixels.
  int si2 = std::min(si + 1, m_width - 1);
  int ti2 = std::min(ti + 1, m_height - 1);
  Pixel::Composite c1 = PixelAt(si, ti);
  Pixel::Composite c2 = PixelAt(si2, ti);
  Pixel::Composite c3 = PixelAt(si, ti2);
  Pixel::Composite c4 = PixelAt(si2, ti2);

  // Linear interpolation along s-axis.
  Pixel::Composite cs1 = Lerp(c1, c2, sw);
  Pixel::Composite cs2 = Lerp(c3, c4, sw);

  // Linear interpolation along t-axis.
  return Lerp(cs1, cs2, tw);
}

