// -*- Mode: c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//------------------------------------------------------------------------------
// Copyright (c) 2014, Marcus Geelnard
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

#ifndef MAGERAY_EXPRESSION_PARSER_H_
#define MAGERAY_EXPRESSION_PARSER_H_

#include <stdexcept>
#include <string>
#include <sstream>

#include "base/types.h"
#include "vec.h"

namespace mageray {

class ExpressionParser {
  public:
    class error : public std::exception {
      public:
        error(const std::string& expression, const std::string& msg) noexcept {
          m_expression = expression;
          m_msg = msg;
        }

        virtual ~error() noexcept {}

        virtual const char* what() const noexcept {
          return "An expression parser error occured.";
        }

        std::string message() const {
          std::ostringstream ss;
          ss << "When parsing \"" << m_expression << "\": " << m_msg;
          return ss.str();
        }

      private:
        std::string m_expression;
        std::string m_msg;
    };

    ExpressionParser();
    ~ExpressionParser();

    void SetTime(const scalar t) {
      m_t = t;
    }

    bool ToBool(const char* str);
    int ToInt(const char* str);
    scalar ToScalar(const char* str);
    vec2 ToVec2(const char* str);
    vec3 ToVec3(const char* str);

  private:
    scalar NextPart(const char* str, int& pos);

    // Forward declare this part since it contains exprtk specific data, and
    // we do not want to include exprtk.hpp in this header file.
    class Parser;

    Parser* m_parser;

    scalar m_t;

    FORBID_COPY(ExpressionParser);
};

} // namespace mageray

#endif // MAGERAY_EXPRESSION_PARSER_H_
