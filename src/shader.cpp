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

#include "shader.h"

#include "light.h"
#include "material.h"

//------------------------------------------------------------------------------
// Null shader (cheap, constant result).
//------------------------------------------------------------------------------

vec3 NullShader::LightContribution(
    const SurfaceParameters&,
    const LightParameters&) const {
  return vec3(0.0);
}

vec3 NullShader::ShadeColor(
    const SurfaceParameters&,
    const vec3&) const {
  return vec3(0.0);
}


//------------------------------------------------------------------------------
// Classic phong shader.
//------------------------------------------------------------------------------

vec3 PhongShader::LightContribution(const SurfaceParameters& sp,
    const LightParameters& lp) const {
  // For convenience: extract parameters into local variables.
  const Material* material = sp.material;
  const Light* light = lp.light;
  vec3 view_dir = sp.view_dir;
  vec3 light_dir = lp.dir;
  vec3 normal = sp.normal;
  scalar cos_alpha = lp.cos_alpha;

  // Diffuse contribution.
  scalar light_factor = cos_alpha * material->Diffuse();

  // Specular contribution.
  if (material->Specular() > 0.0) {
    vec3 light_reflect_dir = light_dir - normal * (2.0 * normal.Dot(light_dir));

    light_factor += material->Specular() *
        std::pow(view_dir.Dot(light_reflect_dir), material->Hardness());
  }

  // Diffuse and specular contribution.
  return light->Color() * (light_factor * lp.amount);
}

vec3 PhongShader::ShadeColor(const SurfaceParameters& sp,
    const vec3& lc) const {
  return sp.material->Color() * lc + vec3(sp.material->Ambient());
}
