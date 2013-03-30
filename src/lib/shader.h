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

#ifndef MAGERAY_SHADER_H_
#define MAGERAY_SHADER_H_

#include "base/types.h"
#include "vec.h"

namespace mageray {

class Material;
class Light;

class Shader {
  public:
    virtual ~Shader() {}

    struct SurfaceParam {
        const Material* material; ///< Object material.

        vec3 position;        ///< World space position.
        vec3 normal;          ///< World space surface normal.
        vec2 uv;              ///< UV coordinate.
        vec3 view_dir;        ///< View direction.
    };

    struct MaterialParam {
        vec3 diffuse;         ///< Diffuse color.
        scalar alpha;         ///< Opacity.
        vec3 specular;        ///< Specular color.
        vec3 normal;          ///< World space surface normal.
    };

    struct LightParam {
        const Light* light;   ///< Light source.

        vec3 dir;             ///< Direction from position to the light.
        scalar dist;          ///< Distance from position to the light.
        scalar cos_alpha;     ///< cos(angle between light dir and normal).
        scalar amount;        ///< 1.0 for non-shadowed, 0.0 for shadowed.
    };

    /// Calculate surface material.
    /// @param[in] sp Surface paramters.
    /// @param[out] mp Material parameters (all fields that are used in
    /// subsequent shader passes have to filled out).
    virtual void MaterialPass(const SurfaceParam& sp,
        MaterialParam& mp) const = 0;

    /// Light contribtions for a single light source.
    /// @param[in] sp Surface paramters.
    /// @param[in] mp Material parameters.
    /// @param[in] lp Light parameters.
    /// @returns The total light contribution from the given light source.
    virtual vec3 LightPass(const SurfaceParam& sp,
        const MaterialParam& mp, const LightParam& lp) const = 0;

    /// Final color shading.
    /// @param[in] sp Surface paramters.
    /// @param[in] mp Material parameters.
    /// @param[in] lc Total light contribution.
    /// @returns Final color.
    virtual vec3 FinalPass(const SurfaceParam& sp,
        const MaterialParam& mp,
        const vec3& lc) const = 0;
};

class NullShader : public Shader {
  public:
    virtual void MaterialPass(const SurfaceParam&,
        MaterialParam&) const;

    virtual vec3 LightPass(const SurfaceParam&,
        const MaterialParam&, const LightParam&) const;

    virtual vec3 FinalPass(const SurfaceParam&,
        const MaterialParam&, const vec3&) const;
};

class PhongShader : public Shader {
  public:
    virtual void MaterialPass(const SurfaceParam& sp,
        MaterialParam& mp) const;

    virtual vec3 LightPass(const SurfaceParam& sp,
        const MaterialParam& mp, const LightParam& lp) const;

    virtual vec3 FinalPass(const SurfaceParam& sp,
        const MaterialParam&, const vec3& lc) const;
};

} // namespace mageray

#endif // MAGERAY_SHADER_H_
