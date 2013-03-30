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

#ifndef MAGERAY_CAMERA_H_
#define MAGERAY_CAMERA_H_

#include <cmath>

#include "base/types.h"
#include "vec.h"
#include "mat.h"

namespace mageray {

class Camera {
  public:
    Camera() {
      Reset();
    }
    ~Camera() {}

    void Reset();

    /// Get the camera position.
    const vec3& Position() const {
      return m_position;
    }

    /// Get the camera forward direction.
    /// The forward direction vector is scaled to match the field of view (FOV)
    /// of the camera.
    vec3 Forward() const {
      scalar len = scalar(1.0) / std::tan(m_fov * (PI / scalar(360.0)));
      return m_matrix.TransformDirection(vec3(0, 1, 0) * len);
    }

    /// Get the camera right axis.
    vec3 Right() const {
      return m_matrix.TransformDirection(vec3(1, 0, 0));
    }

    /// Get the camera up axis.
    vec3 Up() const {
      return m_matrix.TransformDirection(vec3(0, 0, 1));
    }

    /// Set the camera position.
    void SetPosition(const vec3& position) {
      m_position = position;
      UpdateMatrices();
    }

    /// Set the observation point.
    void SetLookAt(const vec3& look_at) {
      m_look_at = look_at;
      UpdateMatrices();
    }

    /// Set the nominal up direction.
    void SetNominalUp(const vec3& up) {
      m_nominal_up = up;
      UpdateMatrices();
    }

    void SetFOV(const scalar& fov) {
      m_fov = fov;
    }

    /// Get the transformation matrix.
    const mat3x4 Matrix() const {
      return m_matrix;
    }

    /// Get the inverse transformation matrix.
    const mat3x4 InvMatrix() const {
      return m_inv_matrix;
    }

  private:
    /// Update the transformation matrices.
    void UpdateMatrices();

    vec3 m_position;
    vec3 m_look_at;
    vec3 m_nominal_up;

    scalar m_fov;

    mat3x4 m_matrix;
    mat3x4 m_inv_matrix;
};

} // namespace mageray

#endif // MAGERAY_CAMERA_H_
