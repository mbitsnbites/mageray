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
#include "sampler.h"

namespace mageray {

//------------------------------------------------------------------------------
// Null shader (cheap, constant result).
//------------------------------------------------------------------------------

void NullShader::MaterialPass(const SurfaceParam&, MaterialParam&) const {
}

vec3 NullShader::LightPass(const SurfaceParam&, const MaterialParam&,
    const LightParam&) const {
  return vec3(0.0);
}

vec3 NullShader::FinalPass(const SurfaceParam&, const MaterialParam&,
    const vec3&) const {
  return vec3(0.0);
}


//------------------------------------------------------------------------------
// Classic phong shader.
//------------------------------------------------------------------------------

void PhongShader::MaterialPass(const SurfaceParam& sp,
    MaterialParam& mp) const {
  // Diffuse color.
  mp.diffuse = sp.material->Diffuse();
  mp.transparency = sp.material->Transparency();
  if (sp.material->DiffuseMap().HasImage()) {
    Pixel c = sp.material->DiffuseMap().Sample(sp.uv);
    vec3 color = vec3(c.r(), c.g(), c.b()) * scalar(1.0 / 255.0);
    mp.diffuse = mp.diffuse * color;
    mp.transparency = mp.transparency * color * ((255 - c.a()) * scalar(1.0 / 255.0));
  }

  // Specular color.
  mp.specular = sp.material->Specular();
  if (sp.material->SpecularMap().HasImage()) {
    Pixel c = sp.material->SpecularMap().Sample(sp.uv);
    mp.specular = mp.specular * vec3(c.r(), c.g(), c.b()) * scalar(1.0 / 255.0);
  }

  // Normal (possibly modified by a normal map).
  // TODO(mage): Implement normal map.
  mp.normal = sp.normal;
}

vec3 PhongShader::LightPass(const SurfaceParam& sp,
    const MaterialParam& mp, const LightParam& lp) const {
  // Diffuse contribution.
  vec3 light_factor = mp.diffuse * lp.cos_alpha;

  // Specular contribution.
  vec3 light_reflect_dir = lp.dir - mp.normal * (scalar(2.0) *
      mp.normal.Dot(lp.dir));

  light_factor += mp.specular *
      std::pow(sp.view_dir.Dot(light_reflect_dir), sp.material->Hardness());

  // Light falloff, depnding on distance.
  scalar falloff = lp.light->Distance() / (lp.light->Distance() + lp.dist);

  // Diffuse and specular contribution.
  return lp.light->Color() * light_factor * (lp.amount * falloff);
}

vec3 PhongShader::FinalPass(const SurfaceParam& sp,
    const MaterialParam&, const vec3& lc) const {
  return lc + vec3(sp.material->Ambient());
}

} // namespace mageray
