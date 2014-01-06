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

#include <exprtk.hpp>

#include "expression_parser.h"

namespace mageray {

class ExpressionParser::Parser {
  public:
    Parser() {
      m_symbol_table.add_constants();
      m_expression.register_symbol_table(m_symbol_table);
    }

    void AddVariable(const std::string& name, scalar& var) {
      m_symbol_table.add_variable(name, var);
    }

    scalar Evaluate(const std::string& str) {
      if (!m_parser.compile(str, m_expression)) {
        throw error(str, m_parser.error());
      }
      return m_expression.value();
    }

  private:
    exprtk::symbol_table<scalar> m_symbol_table;
    exprtk::expression<scalar> m_expression;
    exprtk::parser<scalar> m_parser;
};

ExpressionParser::ExpressionParser() : m_t(0.0) {
  m_parser = new Parser;
  m_parser->AddVariable("t", m_t);
}

ExpressionParser::~ExpressionParser() {
  delete m_parser;
}

bool ExpressionParser::ToBool(const char* str) {
  scalar s;
  int pos = 0;
  s = NextPart(str, pos);
  return s != scalar(0.0);
}

int ExpressionParser::ToInt(const char* str) {
  scalar s;
  int pos = 0;
  s = NextPart(str, pos);
  return static_cast<int>(s);
}

scalar ExpressionParser::ToScalar(const char* str) {
  scalar s;
  int pos = 0;
  s = NextPart(str, pos);
  return s;
}

vec2 ExpressionParser::ToVec2(const char* str) {
  vec2 v;
  int pos = 0;
  v.u = NextPart(str, pos);
  v.v = NextPart(str, pos);
  return v;
}

vec3 ExpressionParser::ToVec3(const char* str) {
  vec3 v;
  int pos = 0;
  v.x = NextPart(str, pos);
  v.y = NextPart(str, pos);
  v.z = NextPart(str, pos);
  return v;
}

scalar ExpressionParser::NextPart(const char* str, int& pos) {
  // Extract next sub-string, up to the next white space.
  std::string expression_string;
  for (; str[pos]; pos++) {
    if (str[pos] == ' ') {
      while (str[pos] == ' ') {
        pos++;
      }
      break;
    }
    expression_string += str[pos];
  }

  // Evaluate string.
  return m_parser->Evaluate(expression_string);
}

} // namespace mageray
