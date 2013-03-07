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

#ifndef RAYMAGE_IMAGE_H_
#define RAYMAGE_IMAGE_H_

#include <istream>
#include <memory>
#include <ostream>

#include "base/log.h"
#include "base/types.h"
#include "pixel.h"

/// An image data container.
class Image {
  public:
    Image() : m_width(0), m_height(0) {}
    ~Image() {}

    /// Reset the image to its default (empty) state.
    void Reset();

    /// Allocate memory for a width x height image.
    /// @param[in] width The width of the image.
    /// @param[in] height The height of the image.
    /// @returns true if the operation succeeded.
    bool Allocate(int width, int height);

    /// Load an image from a stream (PNG or JPEG).
    /// @param[in] stream The stream from which to read the image.
    /// @returns true if the operation succeeded.
    bool Load(std::istream& stream);

    /// Load an image from a file (PNG or JPEG).
    /// @param[in] file_name The name of the file to load.
    /// @returns true if the operation succeeded.
    bool Load(const char* file_name);

    /// Save an image as a PNG file to a stream.
    /// @param[in] stream The stream to which to write the image.
    /// @returns true if the operation succeeded.
    bool SavePNG(std::ostream& stream);

    /// Save an image as a PNG file.
    /// @param[in] file_name The name of the file to save.
    /// @returns true if the operation succeeded.
    bool SavePNG(const char* file_name);

    /// Clear the entire image with a solid color.
    /// @param[in] color The color to fill the image with.
    void Clear(const Pixel& color);

    /// Get a reference to the pixel at the given coordinates.
    /// @param[in] s The s coordinate, in the range [0, width()).
    /// @param[in] t The t coordinate, in the range [0, height()).
    /// @retruns A reference to the pixel, which can be both read and written.
    Pixel& PixelAt(int s, int t) const {
      ASSERT(s >= 0 && s < m_width && t >= 0 && t < m_height,
          "Coordinates out of range (%d,%d).", s, t);
      return m_data.get()[t * m_width + s];
    }

    /// Sample a color value using bilinear interpolation.
    /// @param[in] s The s coordinate, in the range [0,1).
    /// @param[in] t The t coordinate, in the range [0,1).
    /// @returns An interpolated color value.
    Pixel Sample(float s, float t) const;

    /// Check if the image is empty.
    /// @returns true if the image is empty (i.e. has zero pixels).
    bool Empty() const {
      return m_width <= 0 || m_height <= 0;
    }

    /// Get image width.
    /// @returns The width of the image.
    int Width() const {
      return m_width;
    }

    /// Get image height.
    /// @returns The height of the image.
    int Height() const {
      return m_height;
    }

    /// Get image size.
    /// @returns The number of bytes occupied by the image data.
    int Size() const {
      return m_width * m_height * sizeof(Pixel);
    }

  private:
    bool LoadPNG(std::istream& stream);
    bool LoadJPEG(std::istream& stream);

    std::unique_ptr<Pixel> m_data;
    int m_width;
    int m_height;

    float m_s_scale;  // Factor for converting s coordinates to fixed point.
    float m_t_scale;  // Factor for converting t coordinates to fixed point.

    FORBID_COPY(Image);
};

#endif // RAYMAGE_IMAGE_H_
