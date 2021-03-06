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

#if defined(USE_LIBPNG)
# include <png.h>
#elif defined(USE_LODEPNG)
# include <lodepng.h>
#else
# error "Define USE_LIBPNG or USE_LODEPNG!"
#endif

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

  // Compare signature to the read buffer.
  return std::equal(buf, buf + len, signature);
}

#ifdef USE_LIBPNG
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
#endif // USE_LIBPNG

#ifdef USE_LODEPNG
void SwapRedBlue(mageray::Pixel* out, const mageray::Pixel* in, size_t count) {
  while (count--) {
    mageray::Pixel::Composite p = *in++;
    mageray::Pixel::Composite ag = p & 0xff00ff00;
    mageray::Pixel::Composite br = p & 0x00ff00ff;
    *out++ = ag | (br << 16) | (br >> 16);
  }
}
#endif // USE_LODEPNG

}

namespace mageray {

Image* Image::LoadPNG(std::istream& stream) {
  static const unsigned char signature[] = {137, 80, 78, 71, 13, 10, 26, 10};
  if (!MatchingSignature<sizeof(signature)>(stream, signature)) {
    return NULL;
  }

  DLOG("Identified PNG file format.");

#ifdef USE_LIBPNG
  // Initialize PNG data structures.
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
      NULL, NULL);
  if (!png_ptr) {
    return NULL;
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
    return NULL;
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
  Image* image = new Image(width, height);

  // Set up row pointers.
  std::unique_ptr<png_bytep> row_pointers(new png_bytep[height]);
  for (int i = 0; i < height; ++i) {
    row_pointers.get()[i] = reinterpret_cast<png_bytep>(&image->PixelAt(0, i));
  }

  // Read the PNG file to the image data buffer.
  png_read_image(png_ptr, row_pointers.get());

  // Free up resources.
  png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);

  return image;
#endif // USE_LIBPNG

#ifdef USE_LODEPNG
  // Load stream into memory.
  std::streampos start_pos = stream.tellg();
  stream.seekg(0, std::ios_base::end);
  size_t file_size = stream.tellg() - start_pos;
  stream.seekg(start_pos);
  std::vector<unsigned char> data(file_size);
  stream.read(reinterpret_cast<char*>(&data[0]), file_size);

  // Decode the PNG from memory.
  std::vector<unsigned char> raw_data;
  unsigned width, height;
  unsigned error = lodepng::decode(raw_data, width, height, data);
  if (error) {
    return NULL;
  }
  DLOG("PNG size: %d x %d", width, height);

  // Copy the data to the internal image representation.
  // TODO(m): Decode directly to internal memory.
  Image* image = new Image(width, height);
  SwapRedBlue(&image->PixelAt(0, 0), reinterpret_cast<mageray::Pixel*>(&raw_data[0]), width * height);

  return image;
#endif // USE_LODEPNG
}

Image* Image::LoadJPEG(std::istream& stream) {
  static const unsigned char signature[] = {255, 216};
  if (!MatchingSignature<sizeof(signature)>(stream, signature)) {
    return NULL;
  }

  DLOG("Identified JPEG file format.");

  // TODO(m): Implement me!
  LOG("JPEG reading support not yet implemented.");
  return NULL;
}


Image* Image::Load(std::istream& stream) {
  // Try different file formats.
  Image* image = LoadPNG(stream);
  if (image) {
    return image;
  }
  image = LoadJPEG(stream);

  return image;
}

Image* Image::Load(const char* file_name) {
  std::ifstream is(file_name, std::ios_base::in | std::ios_base::binary);
  if (is.good()) {
    return Load(is);
  }

  LOG("Unable to open image file %s.", file_name);
  return NULL;
}

bool Image::SavePNG(std::ostream& stream) const {
#ifdef USE_LIBPNG
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
  png_write_end(png_ptr, NULL);

  // Free up resources.
  png_destroy_write_struct(&png_ptr, &info_ptr);
#endif // USE_LIBPNG

#ifdef USE_LODEPNG
  // Swap R & G.
  std::vector<Pixel> pixels(m_width * m_height);
  SwapRedBlue(&pixels[0], m_data.get(), m_width * m_height);

  // Encode to an in-memory array.
  std::vector<unsigned char> data;
  unsigned error = lodepng::encode(data, reinterpret_cast<unsigned char*>(&pixels[0]), m_width, m_height);
  if (error) {
    return false;
  }

  // Write to the output stream.
  stream.write(reinterpret_cast<char*>(&data[0]), data.size());
#endif // USE_LODEPNG

  return true;
}

bool Image::SavePNG(const char* file_name) const {
  std::ofstream os(file_name, std::ios_base::out | std::ios_base::binary);
  if (os.good()) {
    return SavePNG(os);
  }
  return false;
}

} // namespace mageray
