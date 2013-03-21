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

#ifndef MAGERAY_TREE_H_
#define MAGERAY_TREE_H_

#include <memory>
#include <vector>

#include "aabb.h"
#include "base/log.h"
#include "base/types.h"
#include "hitinfo.h"
#include "mesh_data.h"
#include "vec.h"
#include "ray.h"
#include "triangle.h"

/// Generic tree node class.
/// @note This class is intended to be extended.
class Node {
  public:
    Node() {}

    ~Node() {
      if (IsLeafNode()) {
        return;
      }
      delete FirstChild();
      delete SecondChild();
    }

    /// @returns The axis aligned bounding box for this node.
    AABB& BoundingBox() {
      return m_aabb;
    }

    /// @returns True if this is a leaf node.
    bool IsLeafNode() const {
      return m_leaf.dummy == NULL;
    }

    /// @returns The first child node of a branch node.
    Node* FirstChild() const {
      ASSERT(!IsLeafNode(), "This is a leaf node.");
      return m_branch.first;
    }

    /// @returns The second child node of a branch node.
    Node* SecondChild() const {
      ASSERT(!IsLeafNode(), "This is a leaf node.");
      return m_branch.second;
    }

    /// Set the child nodes of this branch node.
    void SetChildren(Node* first, Node* second) {
      m_branch.first = first;
      m_branch.second = second;
    }

  protected:
    AABB m_aabb;

    union {
      struct {
        Node* first;
        Node* second;
      } m_branch;
      struct {
        const void* item;
        void* dummy;
      } m_leaf;
    };

    // TODO(mage): We need to change the ownership of nodes so that we don't
    // do anything dangerous in ~Node() - now we can potentially copy a node
    // and then delete it's children twice.
//    FORBID_COPY(Node);
};

/// Template tree node class that supports leaf nodes.
template <class T>
class TypedNode : public Node {
  public:
    /// Set the item of a leaf node.
    void SetItem(const T* item) {
      m_leaf.item = item;
      m_leaf.dummy = NULL;
    }

    /// @returns The data item of a leaf node.
    const T* Item() const {
      ASSERT(IsLeafNode(), "This is not a leaf node.");
      return static_cast<const T*>(m_leaf.item);
    }
};

/// Generic binary axis aligned bounding box tree.
/// @note This class is intended to be extended.
class Tree {
  public:
    Tree() {}

    /// @returns The bounding box for the tree.
    const AABB& BoundingBox() {
      ASSERT(m_root.get() != NULL, "The tree is undefined.");
      return m_root->BoundingBox();
    }

    /// Find intersection between tree and ray.
    /// @param ray The ray to shoot into the tree.
    /// @param[in,out] hit Current closest hit information.
    /// @returns True if the ray intersects with a primitive in the tree.
    bool Intersect(const Ray& ray, HitInfo& hit);

  protected:
    /// Build a bounding box tree.
    /// @param leaves The leaf nodes to construct the tree from.
    /// @param aabb The total bounding box for all the nodes.
    void Build(std::vector<Node*>& leaves, const AABB& aabb);

    /// Recursively intersect a ray against a sub tree.
    /// @param node The root node of the sub tree to intersect.
    /// @param ray The ray to shoot into the tree.
    /// @param[in,out] hit Current closest hit information.
    /// @returns True if the ray intersects with a primitive in the tree.
    /// @note This method assumes that the bounding box for the node has
    /// already been positively checked for intersection with the ray.
    virtual bool RecursiveIntersect(const Node* node, const Ray& ray,
        HitInfo& hit) const = 0;

    // All nodes (except the leaf nodes) in the tree are managed by the root
    // node.
    std::unique_ptr<Node> m_root;

    // The leaf nodes are stored directly in an array, and referenced from
    // the branch nodes in the tree.
    std::vector<Node> m_leaf_nodes;

    FORBID_COPY(Tree);
};

/// Triangle tree node class.
typedef TypedNode<Triangle> TriangleNode;

/// Binary axis aligned bounding box tree for triangles.
class TriangleTree : public Tree {
  public:
    /// Build a triangle tree from raw mesh data.
    /// @param triangles An array of the triangles to put into the tree.
    /// @param vertices The vertex array used by the triangles.
    void Build(const MeshData& data);

  protected:
    virtual bool RecursiveIntersect(const Node* node, const Ray& ray,
       HitInfo& hit) const;

  private:
    /// Check intersection between ray and triangle.
    /// @param ray The ray.
    /// @param triangle The triangle to intersect.
    /// @param[in,out] hit Current closest hit information.
    /// @returns True if the ray intersects with the triangle.
    bool IntersectTriangle(const Ray& ray, const Triangle* triangle,
        HitInfo& hit) const;

    const MeshData* m_data;
};

#endif // MAGERAY_TREE_H_
