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
#include <future>
#include <thread>

#include "base/perf.h"

namespace {

/// Calculate bounding box union for a range of nodes.
void BoundingBoxUnion(AABB& aabb, std::vector<Node*>::const_iterator start,
    std::vector<Node*>::const_iterator stop) {
  ASSERT(start < stop, "Empty union operation.");
  aabb = (*start)->BoundingBox();
  for (std::vector<Node*>::const_iterator it = start + 1; it != stop; ++it) {
    aabb += (*it)->BoundingBox();
  }
}

/// Build a sub-tree from an array of leaf nodes.
Node* BuildSubtree(const AABB& aabb, std::vector<Node*>::iterator start,
    std::vector<Node*>::iterator stop, int threaded_depth) {
  if ((stop - start) == 1) {
    // Return the leaf node.
    return *start;
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
  scalar mid2 = aabb[min_bound] + aabb[max_bound];

  // Sort the leaves array into two new sub-arrays:
  //  first sub-array: start to start_second (exclusive)
  //  second sub-array: start_second to stop (exclusive)
  std::vector<Node*>::iterator start_second = stop;
  for (std::vector<Node*>::iterator it = start; it < start_second;) {
    Node* leaf = *it;

    // Check if this leaf is below or above the split threshold.
    const AABB& leaf_aabb = leaf->BoundingBox();
    scalar leaf_mid2 = leaf_aabb[min_bound] + leaf_aabb[max_bound];
    if (leaf_mid2 < mid2) {
      // Keep the leaf in the first half.
      it++;
    } else {
      // Swap in the leaf into the second half.
      start_second--;
      Node* tmp = *start_second;
      *start_second = leaf;
      *it = tmp;
    }
  }

  // Handle degenerate cases (we must have at least one element in each array).
  if (start_second == start) {
    start_second++;
  } else if (start_second == stop) {
    start_second--;
  }

  // Calculate new bounding boxes for the first/second array.
  AABB first_aabb, second_aabb;
  BoundingBoxUnion(first_aabb, start, start_second);
  BoundingBoxUnion(second_aabb, start_second, stop);

  // Recursively build new sub-trees.
  Node* first_branch;
  Node* second_branch;
  if (threaded_depth == 1) {
    // Spawn new threads when we're at the correct depth to better utilize
    // multi-core CPUs.
    std::promise<Node*> result1;
    std::promise<Node*> result2;
    std::thread t1 = std::thread([&] {
        result1.set_value(BuildSubtree(first_aabb, start, start_second, threaded_depth - 1));
    });
    std::thread t2 = std::thread([&] {
        result2.set_value(BuildSubtree(second_aabb, start_second, stop, threaded_depth - 1));
    });
    first_branch = result1.get_future().get();
    second_branch = result2.get_future().get();
    t1.join();
    t2.join();
  } else {
    first_branch = BuildSubtree(first_aabb, start, start_second, threaded_depth - 1);
    second_branch = BuildSubtree(second_aabb, start_second, stop, threaded_depth - 1);
  }

  // Create a new tree branch node, and return it.
  return new Node(aabb, first_branch, second_branch);
}

}

void Tree::Build(std::vector<Node*>& leaves, const AABB& aabb) {
  if (leaves.size() == 0) {
    LOG("Empty tree (no nodes to process).");
    m_root.reset(NULL);
    return;
  }

  // Select at which tree depth to split execution into parallell threads.
  // threaded_depth interpretation:
  //   0 -> Single threaded
  //   1 -> 2 threads
  //   2 -> 4 threads
  //   3 -> 8 threads
  //   etc.
  int threaded_depth;
  int concurrency = std::thread::hardware_concurrency();
  if (concurrency == 0) {
    // NOTE: hardware_concurrency() in gcc 4.6.3 always returns zero :(
    concurrency = 2;
  }
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
  m_root.reset(BuildSubtree(aabb, leaves.begin(), leaves.end(), threaded_depth));
}

void TriangleTree::Build(const std::vector<Triangle>& triangles,
    const std::vector<Vertex>& vertices) {
  ScopedPerf _perf = ScopedPerf(__FUNCTION__);

  // We keep a reference to the vertices.
  m_vertices = &vertices;

  ScopedPerf _leaf_perf = ScopedPerf("Generate leaf nodes");

  // Create a vector of nodes, and calculate the total bounding box while we're
  // at it...
  AABB aabb;
  std::vector<Node*> leaves(triangles.size());
  std::vector<Node*>::iterator node_it = leaves.begin();
  std::vector<Triangle>::const_iterator tri_it;
  for (tri_it = triangles.begin(); tri_it != triangles.end(); tri_it++) {
    // Get the triangle.
    const Triangle* triangle = &(*tri_it);

    // Get the three vertex positions for the triangle.
    const vec3 p1 = vertices[triangle->a].position;
    const vec3 p2 = vertices[triangle->b].position;
    const vec3 p3 = vertices[triangle->c].position;

    // Calculate the bounding box for the triangle.
    AABB tri_aabb(std::min(p1.x, std::min(p2.x, p3.x)),
                  std::min(p1.y, std::min(p2.y, p3.y)),
                  std::min(p1.z, std::min(p2.z, p3.z)),
                  std::max(p1.x, std::max(p2.x, p3.x)),
                  std::max(p1.y, std::max(p2.y, p3.y)),
                  std::max(p1.z, std::max(p2.z, p3.z)));

    // Create a new tree leaf node, and add it to the vector.
    TriangleNode* node = new TriangleNode(tri_aabb, triangle);
    *node_it++ = node;

    // Update the union bounding box for all the triangles.
    if (tri_it == triangles.begin()) {
      aabb = node->BoundingBox();
    } else {
      aabb += node->BoundingBox();
    }
  }

  _leaf_perf.Done();

  // Build the tree.
  Tree::Build(leaves, aabb);

  _perf.Done();
}
