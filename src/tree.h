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

#ifndef RAYMAGE_TREE_H_
#define RAYMAGE_TREE_H_

#include <vector>

#include "base/log.h"
#include "base/types.h"
#include "vec.h"
#include "triangle.h"

/// Axis aligned bounding box.
struct AABB {
  AABB(const vec3& min, const vec3& max) {
    bounds[XMIN] = min.x;
    bounds[YMIN] = min.y;
    bounds[ZMIN] = min.z;
    bounds[XMAX] = max.x;
    bounds[YMAX] = max.y;
    bounds[ZMAX] = max.z;
  }

  vec3 min() const {
    return vec3(bounds[XMIN], bounds[YMIN], bounds[ZMIN]);
  }

  vec3 max() const {
    return vec3(bounds[XMAX], bounds[YMAX], bounds[ZMAX]);
  }

  static const int XMIN = 0;
  static const int YMIN = 1;
  static const int ZMIN = 2;
  static const int XMAX = 3;
  static const int YMAX = 4;
  static const int ZMAX = 5;

  scalar bounds[6];
};

/// Tree node class.
template <class T>
class Node {
  public:
    ~Node() {
      if (IsLeafNode()) {
        delete Item();
        return;
      }
      delete ChildA();
      delete ChildB();
    }

    /// Constructor for leaf nodes.
    Node(const vec3& min, const vec3& max, T* item) :
        m_aabb(min, max), m_first(item), m_second(NULL) {}

    /// Constructor for branch nodes.
    Node(const vec3& min, const vec3& max, Node* first, Node* second) :
        m_aabb(min, max), m_first(first), m_second(second) {}

    /// @returns The axis aligned bounding box for this node.
    const AABB& BoundingBox() const {
      return m_aabb;
    }

    /// @returns True if this is a leaf node.
    bool IsLeafNode() const {
      return m_second == NULL;
    }

    /// @returns The first child node of a branch node.
    Node* ChildA() const {
      ASSERT(!IsLeafNode(), "This is a leaf node.");
      return static_cast<Node*>(m_first);
    }

    /// @returns The second child node of a branch node.
    Node* ChildB() const {
      ASSERT(!IsLeafNode(), "This is a leaf node.");
      return static_cast<Node*>(m_second);
    }

    /// @returns The data item of a leaf node.
    T* Item() const {
      ASSERT(IsLeafNode(), "This is not a leaf node.");
      return static_cast<T*>(m_first);
    }

  private:
    AABB m_aabb;
    void* m_first;
    void* m_second;
};

/// Binary axis aligned bounding box tree for triangles.
class TriangleTree {
  public:
    TriangleTree() : m_root(NULL) {}
    ~TriangleTree();

    /// Build a triangle tree from raw mesh data.
    void Build(const std::vector<Triangle>& triangles,
        const std::vector<Vertex>& vertices);

  private:
    Node<Triangle>* m_root;
};

#endif // RAYMAGE_TREE_H_
