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

#ifndef MAGERAY_MATERIAL_H_
#define MAGERAY_MATERIAL_H_

#include "vec.h"
#include "sampler.h"

namespace mageray {

class Image;
class Shader;

class Material {
  public:
    Material() {
      Reset();
    }
    ~Material() {}

    void Reset();

    void SetAmbient(const vec3& ambient) {
      m_ambient = ambient;
    }

    const vec3& Ambient() const {
      return m_ambient;
    }

    void SetDiffuse(const vec3& diffuse) {
      m_diffuse = diffuse;
    }

    const vec3& Diffuse() const {
      return m_diffuse;
    }

    void SetSpecular(const vec3& specular) {
      m_specular = specular;
    }

    const vec3& Specular() const {
      return m_specular;
    }

    void SetTransparency(const vec3& transparency) {
      m_transparency = transparency;
    }

    const vec3& Transparency() const {
      return m_transparency;
    }

    void SetMirror(const bool mirror) {
      m_mirror = mirror;
    }

    bool Mirror() const {
      return m_mirror;
    }

    void SetHardness(const scalar& hardness) {
      m_hardness = hardness;
    }

    const scalar& Hardness() const {
      return m_hardness;
    }

    void SetIor(const scalar& ior) {
      m_ior = ior;
    }

    const scalar& Ior() const {
      return m_ior;
    }

    const mageray::Shader* Shader() const {
      return m_shader;
    }

    void SetShader(mageray::Shader* shader) {
      m_shader = shader;
    }

    const Sampler& DiffuseMap() const {
      return m_diffuse_map;
    }

    Sampler& DiffuseMap() {
      return m_diffuse_map;
    }

    const Sampler& SpecularMap() const {
      return m_specular_map;
    }

    Sampler& SpecularMap() {
      return m_specular_map;
    }

    const Sampler& NormalMap() const {
      return m_normal_map;
    }

    Sampler& NormalMap() {
      return m_normal_map;
    }

  private:
    vec3 m_ambient;
    vec3 m_diffuse;
    vec3 m_specular;
    vec3 m_transparency;
    bool m_mirror;
    scalar m_hardness;
    scalar m_ior;

    Sampler m_diffuse_map;
    Sampler m_specular_map;
    Sampler m_normal_map;

    mageray::Shader* m_shader;
};

} // namespace mageray

#endif // MAGERAY_MATERIAL_H_
