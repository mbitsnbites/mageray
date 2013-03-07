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

#include <memory>
#include <vector>

#include "base/log.h"
#include "base/types.h"
#include "vec.h"
#include "triangle.h"

/// Axis aligned bounding box.
struct AABB {
  enum Axis {
    X = 0,
    Y = 1,
    Z = 2
  };

  enum Bound {
    XMIN = 0,
    YMIN = 1,
    ZMIN = 2,
    XMAX = 3,
    YMAX = 4,
    ZMAX = 5
  };

  scalar bounds[6];

  AABB() {}

  AABB(const vec3& min, const vec3& max) {
    bounds[XMIN] = min.x;
    bounds[YMIN] = min.y;
    bounds[ZMIN] = min.z;
    bounds[XMAX] = max.x;
    bounds[YMAX] = max.y;
    bounds[ZMAX] = max.z;
  }

  /// Minimum coordinate for this bounding box.
  vec3 Min() const {
    return vec3(bounds[XMIN], bounds[YMIN], bounds[ZMIN]);
  }

  /// Maxium coordinate for this bounding box.
  vec3 Max() const {
    return vec3(bounds[XMAX], bounds[YMAX], bounds[ZMAX]);
  }

  /// Sum of Min() and Max().
  vec3 Sum() const {
    return vec3(bounds[XMIN] + bounds[XMAX], bounds[YMIN] + bounds[YMAX],
        bounds[ZMIN] + bounds[ZMAX]);
  }

  /// Maxium side axis.
  Axis LargestAxis() const {
    scalar dx = bounds[XMAX] - bounds[XMIN];
    scalar dy = bounds[YMAX] - bounds[YMIN];
    scalar dz = bounds[ZMAX] - bounds[ZMIN];
    if (dx > dy) {
      return dx > dz ? X : Z;
    } else {
      return dy > dz ? Y : Z;
    }
  };

  /// Union of two bounding boxes.
  AABB& operator+=(const AABB& other) {
    if (other.bounds[XMIN] < bounds[XMIN]) bounds[XMIN] = other.bounds[XMIN];
    if (other.bounds[YMIN] < bounds[YMIN]) bounds[YMIN] = other.bounds[YMIN];
    if (other.bounds[ZMIN] < bounds[ZMIN]) bounds[ZMIN] = other.bounds[ZMIN];
    if (other.bounds[XMAX] > bounds[XMAX]) bounds[XMAX] = other.bounds[XMAX];
    if (other.bounds[YMAX] > bounds[YMAX]) bounds[YMAX] = other.bounds[YMAX];
    if (other.bounds[ZMAX] > bounds[ZMAX]) bounds[ZMAX] = other.bounds[ZMAX];
    return *this;
  }
};

/// Tree node class.
template <class T>
class Node {
  public:
    ~Node() {
      if (IsLeafNode()) {
        return;
      }
      delete FirstChild();
      delete SecondChild();
    }

    /// Constructor for leaf nodes.
    Node(const vec3& min, const vec3& max, const T* item) :
        m_aabb(min, max) {
      m_leaf.item = item;
      m_leaf.dummy = NULL;
    }

    /// Constructor for branch nodes.
    Node(const vec3& min, const vec3& max, Node* first, Node* second) :
        m_aabb(min, max) {
      m_branch.first = first;
      m_branch.second = second;
    }

    /// @returns The axis aligned bounding box for this node.
    const AABB& BoundingBox() const {
      return m_aabb;
    }

    /// @returns True if this is a leaf node.
    bool IsLeafNode() const {
      return m_leaf.dummy == NULL;
    }

    /// @returns The first child node of a branch node.
    Node<T>* FirstChild() const {
      ASSERT(!IsLeafNode(), "This is a leaf node.");
      return m_branch.first;
    }

    /// @returns The second child node of a branch node.
    Node<T>* SecondChild() const {
      ASSERT(!IsLeafNode(), "This is a leaf node.");
      return m_branch.second;
    }

    /// @returns The data item of a leaf node.
    const T* Item() const {
      ASSERT(IsLeafNode(), "This is not a leaf node.");
      return m_leaf.item;
    }

  private:
    AABB m_aabb;

    union {
      struct {
        Node<T>* first;
        Node<T>* second;
      } m_branch;
      struct {
        const T* item;
        void* dummy;
      } m_leaf;
    };

    FORBID_COPY(Node);
};

/// Binary axis aligned bounding box tree for triangles.
class TriangleTree {
  public:
    /// Build a triangle tree from raw mesh data.
    void Build(const std::vector<Triangle>& triangles,
        const std::vector<Vertex>& vertices);

  private:
    std::unique_ptr<Node<Triangle> > m_root;
    const std::vector<Vertex>* m_vertices;

    FORBID_COPY(TriangleTree);
};

#endif // RAYMAGE_TREE_H_
