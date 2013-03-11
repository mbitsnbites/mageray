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
#include <thread>

#include "base/perf.h"

namespace {

/// Calculate bounding box union for a list of nodes.
void BoundingBoxUnion(AABB& aabb, const std::vector<Node*>& nodes, int start,
    int stop) {
  ASSERT(start < stop, "Empty union operation.");
  aabb = nodes[start]->BoundingBox();
  for (int i = start + 1; i != stop; ++i) {
    aabb += nodes[i]->BoundingBox();
  }
}

Node* BuildSubtree(const AABB& aabb, std::vector<Node*>& leaves,
    int start, int stop, int depth);

/// A threaded subtree builder.
/// When a SubtreeBuilder object is created, it starts a new thread where the actual
/// processing is done.
class SubtreeBuilder {
  public:
    SubtreeBuilder(const AABB& aabb, std::vector<Node*>& leaves,
        int start, int stop, int depth) :
        m_aabb(&aabb),
        m_leaves(&leaves),
        m_start(start),
        m_stop(stop),
        m_depth(depth) {
      m_thread = std::thread(&SubtreeBuilder::Run, this);
    }

    /// Get the result (blocking).
    /// Wait for the thread to finish and retrieve the result.
    Node* GetResult() {
      m_thread.join();
      return m_result;
    }

  private:
    void Run() {
      m_result = BuildSubtree(*m_aabb, *m_leaves, m_start, m_stop, m_depth);
    }

    const AABB* m_aabb;
    std::vector<Node*>* m_leaves;
    int m_start;
    int m_stop;
    int m_depth;
    Node* m_result;
    std::thread m_thread;
};

/// Build a sub-tree from a list of leaf nodes.
Node* BuildSubtree(const AABB& aabb, std::vector<Node*>& leaves,
    int start, int stop, int threaded_depth) {
  if ((stop - start) == 1) {
    // Return the leaf node.
    return leaves[start];
  }

  // Determine the split direction.
  AABB::Axis axis = aabb.LargestAxis();
  AABB::Bound min_bound, max_bound;
  switch (axis) {
    case AABB::X: {
      min_bound = AABB::XMIN;
      max_bound = AABB::XMAX;
      break;
    }
    case AABB::Y: {
      min_bound = AABB::YMIN;
      max_bound = AABB::YMAX;
      break;
    }
    case AABB::Z: {
      min_bound = AABB::ZMIN;
      max_bound = AABB::ZMAX;
      break;
    }
  }

  // Get the split threshold (2 x middle point along split axis).
  scalar mid2 = aabb.bounds[min_bound] + aabb.bounds[max_bound];

  // Sort the leaves vector into two new (sub-)vectors:
  //  first sub-vector: start to start_second (exclusive)
  //  second sub-vector: start_second to stop (exclusive)
  int start_second = stop;
  for (int i = start; i < start_second;) {
    Node* leaf = leaves[i];

    // Check if this leaf is below or above the split threshold.
    const AABB& leaf_aabb = leaf->BoundingBox();
    scalar leaf_mid2 = leaf_aabb.bounds[min_bound] + leaf_aabb.bounds[max_bound];
    if (leaf_mid2 < mid2) {
      // Keep the leaf in the first half.
      i++;
    } else {
      // Swap in the leaf into the second half.
      start_second--;
      Node* tmp = leaves[start_second];
      leaves[start_second] = leaf;
      leaves[i] = tmp;
    }
  }

  // Handle degenerate cases (we must have at least one element in each vector).
  if (start_second == start) {
    start_second++;
  } else if (start_second == stop) {
    start_second--;
  }

  // Calculate new bounding boxes for the first/second vectors.
  AABB first_aabb, second_aabb;
  BoundingBoxUnion(first_aabb, leaves, start, start_second);
  BoundingBoxUnion(second_aabb, leaves, start_second, stop);

  // Recursively build new sub-trees.
  Node* first_branch;
  Node* second_branch;
  if (threaded_depth == 1) {
    // Spawn new threads when we're at the correct depth to better utilize
    // multi-core CPUs.
    SubtreeBuilder builder1(first_aabb, leaves, start, start_second, threaded_depth - 1);
    SubtreeBuilder builder2(second_aabb, leaves, start_second, stop, threaded_depth - 1);
    first_branch = builder1.GetResult();
    second_branch = builder2.GetResult();
  } else {
    first_branch = BuildSubtree(first_aabb, leaves, start, start_second, threaded_depth - 1);
    second_branch = BuildSubtree(second_aabb, leaves, start_second, stop, threaded_depth - 1);
  }

  // Create a new tree branch node, and return it.
  return new Node(aabb, first_branch, second_branch);
}

}

void TriangleTree::Build(const std::vector<Triangle>& triangles,
    const std::vector<Vertex>& vertices) {
  ScopedPerf _perf = ScopedPerf(__FUNCTION__);

  // Clear the tree.
  m_root.reset(NULL);

  // We keep a reference to the vertices.
  m_vertices = &vertices;

  if (triangles.size() == 0) {
    LOG("No triangles to process.");
    return;
  }

  // Create a vector of nodes, and calculate the total bounding box while we're
  // at it...
  AABB aabb;
  std::vector<Node*> leaves(triangles.size());
  std::vector<Triangle>::const_iterator tri_it;
  std::vector<Node*>::iterator node_it = leaves.begin();
  for (tri_it = triangles.begin(); tri_it != triangles.end(); tri_it++) {
    // Get triangle.
    const Triangle* triangle = &(*tri_it);

    // Get the three vertex positions for the triangle.
    vec3 p1 = vertices[triangle->a].position;
    vec3 p2 = vertices[triangle->b].position;
    vec3 p3 = vertices[triangle->c].position;

    // Calculate the boundingbox limits for the triangle.
    vec3 min(std::min(p1.x, std::min(p2.x, p3.x)),
             std::min(p1.y, std::min(p2.y, p3.y)),
             std::min(p1.z, std::min(p2.z, p3.z)));
    vec3 max(std::max(p1.x, std::max(p2.x, p3.x)),
             std::max(p1.y, std::max(p2.y, p3.y)),
             std::max(p1.z, std::max(p2.z, p3.z)));

    // Create a new tree leaf node, and add it to the list.
    TriangleNode* node = new TriangleNode(min, max, triangle);
    *node_it++ = node;

    // Update the union bounding box for all the triangles.
    if (tri_it == triangles.begin()) {
      aabb = node->BoundingBox();
    } else {
      aabb += node->BoundingBox();
    }
  }

  // Select at which tree depth to split execution into parallell threads.
  // threaded_depth interpretation:
  //   0 -> Single threaded
  //   1 -> 2 threads
  //   2 -> 4 threads
  //   3 -> 8 threads
  //   4 -> 16 threads
  //   etc.
  int threaded_depth;
  int concurrency = std::thread::hardware_concurrency();
  if (concurrency >= 8)
    threaded_depth = 3;
  else if (concurrency >= 4)
    threaded_depth = 2;
  else if (concurrency >= 2)
    threaded_depth = 1;
  else
    threaded_depth = 0;
  DLOG("Hardware concurrency=%d, threaded_depth=%d", concurrency, threaded_depth);

  // Recursively build the tree, and the result is our root node.
  m_root.reset(BuildSubtree(aabb, leaves, 0, leaves.size(), threaded_depth));

  _perf.Done();
}
