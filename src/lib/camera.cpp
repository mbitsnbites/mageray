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

#include "camera.h"

#include "base/log.h"
#include "base/types.h"

namespace mageray {

void Camera::Reset() {
  // Set default properties.
  m_look_at = vec3(0, 0, 0);
  m_position = vec3(1, -10, 1);
  m_nominal_up = vec3(0, 0, 1);
  m_fov = 90.0;

  // Update transformation matrices.
  UpdateMatrices();
}

void Camera::UpdateMatrices() {
  // Forward direction.
  vec3 forward = (m_look_at - m_position).Normalize();
  ASSERT(forward.AbsSqr() > EPSILON, "Undefined direction.");

  // Right direction.
  ASSERT(m_nominal_up.AbsSqr() > EPSILON, "Undefined nominal up direction.");
  vec3 right = forward.Cross(m_nominal_up).Normalize();
  ASSERT(right.AbsSqr() > EPSILON, "Undefined right direction.");

  // Actual up direction (no need to normalize).
  vec3 up = right.Cross(forward);

  // Construct transformation matrix.
  m_matrix = mat3x4(right, forward, up, m_position);

  // ...and the inverse transformation matrix.
  m_inv_matrix = m_matrix.Inverse();
}

} // namespace mageray
