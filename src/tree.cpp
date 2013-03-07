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

#include "tree.h"

#include <algorithm>
#include <list>

namespace {

/// Calculate bounding box union for a list of nodes.
template <class T>
void BoundingBoxUnion(AABB& aabb, const std::list<Node<T>*>& nodes) {
  typename std::list<Node<T>*>::const_iterator it;
  for (it = nodes.begin(); it != nodes.end(); it++) {
    const Node<T>* node = *it;
    if (it == nodes.begin()) {
      aabb = node->BoundingBox();
    } else {
      aabb += node->BoundingBox();
    }
  }
}

/// Build a sub-tree from a list of leaf nodes.
template <class T>
Node<T>* BuildSubtree(const AABB& aabb, const std::list<Node<T>*>& leaves) {
  if (leaves.size() == 1) {
    // Return the leaf node.
    return *leaves.begin();
  }

  // Determine the split direction and middle point.
  AABB::Axis axis = aabb.LargestAxis();
  vec3 mid2 = aabb.Sum();

  // Split the list into two lists (first and second).
  std::list<Node<T>*> first, second;
  typename std::list<Node<T>*>::const_iterator it;
  // TODO(mage): Can we do this efficiently without the outer switch?
  switch (axis) {
    case AABB::X: {
      // Split along the X axis.
      for (it = leaves.begin(); it != leaves.end(); it++) {
        Node<T>* leaf = *it;
        vec3 leaf_mid2 = leaf->BoundingBox().Sum();
        if (leaf_mid2.x < mid2.x) {
          first.push_back(leaf);
        } else {
          second.push_back(leaf);
        }
      }
      break;
    }
    case AABB::Y: {
      // Split along the Y axis.
      for (it = leaves.begin(); it != leaves.end(); it++) {
        Node<T>* leaf = *it;
        vec3 leaf_mid2 = leaf->BoundingBox().Sum();
        if (leaf_mid2.y < mid2.y) {
          first.push_back(leaf);
        } else {
          second.push_back(leaf);
        }
      }
      break;
    }
    case AABB::Z: {
      // Split along the Z axis.
      for (it = leaves.begin(); it != leaves.end(); it++) {
        Node<T>* leaf = *it;
        vec3 leaf_mid2 = leaf->BoundingBox().Sum();
        if (leaf_mid2.z < mid2.z) {
          first.push_back(leaf);
        } else {
          second.push_back(leaf);
        }
      }
      break;
    }
  }

  // Handle degenerate cases (we must have at least one element in each list).
  if (first.empty()) {
    first.push_back(second.back());
    second.pop_back();
  } else if (second.empty()) {
    second.push_back(first.back());
    first.pop_back();
  }

  // Calculate new bounding boxes for the first/second node lists.
  AABB first_aabb, second_aabb;
  BoundingBoxUnion(first_aabb, first);
  BoundingBoxUnion(second_aabb, second);

  // Recursively build the new sub-trees.
  Node<T>* first_branch = BuildSubtree(first_aabb, first);
  Node<T>* second_branch = BuildSubtree(second_aabb, second);

  // Create a new tree branch node, and return it.
  return new Node<T>(aabb.Min(), aabb.Max(), first_branch,
      second_branch);
}

}

void TriangleTree::Build(const std::vector<Triangle>& triangles,
    const std::vector<Vertex>& vertices) {
  // Clear the tree.
  m_root.reset(NULL);

  if (triangles.size() == 0) {
    LOG("No triangles to process.");
    return;
  }

  // We keep a reference to the vertices.
  m_vertices = &vertices;

  // Create a list of nodes, and calculate the total bounding box while we're
  // at it...
  AABB aabb;
  std::list<Node<Triangle>*> leaves;
  std::vector<Triangle>::const_iterator it;
  for (it = triangles.begin(); it != triangles.end(); it++) {
    // Get triangle.
    const Triangle* triangle = &(*it);

    // Get the three vertex positions for the triangle.
    vec3 p1 = vertices[triangle->a].position;
    vec3 p2 = vertices[triangle->b].position;
    vec3 p3 = vertices[triangle->c].position;

    // Calculate the boundingbox for the triangle.
    vec3 min(std::min(p1.x, std::min(p2.x, p3.x)),
             std::min(p1.y, std::min(p2.y, p3.y)),
             std::min(p1.z, std::min(p2.z, p3.z)));
    vec3 max(std::max(p1.x, std::max(p2.x, p3.x)),
             std::max(p1.y, std::max(p2.y, p3.y)),
             std::max(p1.z, std::max(p2.z, p3.z)));

    // Create a new tree leaf node, and add it to the list.
    Node<Triangle>* node = new Node<Triangle>(min, max, triangle);
    leaves.push_back(node);

    // Update the union bounding box for all the triangles.
    if (it == triangles.begin()) {
      aabb = node->BoundingBox();
    } else {
      aabb += node->BoundingBox();
    }
  }

  // Recursively build the tree, and the result is our root node.
  m_root.reset(BuildSubtree(aabb, leaves));
}

