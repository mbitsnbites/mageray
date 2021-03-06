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
#include "base/platform.h"
#include "base/threads.h"
#include "object.h"
#include "triangle.h"
#include "vec.h"

namespace mageray {

namespace {

/// Calculate bounding box union for a range of nodes.
void BoundingBoxUnion(AABB& aabb, std::vector<Node*>& leaves, unsigned start,
    unsigned stop) {
  ASSERT(start < stop, "Empty union operation.");
  aabb = leaves[start]->BoundingBox();
  for (unsigned i = start + 1; i < stop; ++i) {
    aabb += leaves[i]->BoundingBox();
  }
}

/// Find the optimal split direction for the given sub-tree.
vec3::Axis BestSplitDirection(std::vector<Node*>& leaves, const AABB& aabb,
    unsigned start, unsigned stop) {
  // Preliminary best axis is the one with the longest side in the AABB.
  vec3::Axis best_axis = aabb.LargestAxis();

  // Total size of the sub tree.
  unsigned total = stop - start;
  if (total < 5) {
    // Don't bother too with small sub trees.
    return best_axis;
  }

  // Let the split threshold be in the middle of the axis.
  vec3 mid2 = aabb.Min() + aabb.Max();

  // Iterate the sub tree.
  unsigned counts[3];
  counts[vec3::X] = counts[vec3::Y] = counts[vec3::Z] = 0;
  for (unsigned i = start; i < stop; ++i) {
    Node* leaf = leaves[i];

    // Check if this leaf is below or above the split threshold.
    const AABB& leaf_aabb = leaf->BoundingBox();
    if (leaf_aabb.Min().x + leaf_aabb.Max().x < mid2.x) {
      counts[vec3::X]++;
    }
    if (leaf_aabb.Min().y + leaf_aabb.Max().y < mid2.y) {
      counts[vec3::Y]++;
    }
    if (leaf_aabb.Min().z + leaf_aabb.Max().z < mid2.z) {
      counts[vec3::Z]++;
    }
  }

  // Find branch counts for the lightest branch.
  for (unsigned i = vec3::X; i <= vec3::Z; ++i) {
    counts[i] = std::min(counts[i], total - counts[i]);
  }

  // If the preliminarily selected axis is branched so that at most 1/20 (5%)
  // is in the smallest child branch, let's use it.
  if (20 * counts[best_axis] >= total) {
    return best_axis;
  }

  // ...otherwise, select an axis based on how well balanced it is.
  if (counts[vec3::X] > counts[vec3::Y]) {
    best_axis = counts[vec3::X] > counts[vec3::Z] ? vec3::X : vec3::Z;
  } else {
    best_axis = counts[vec3::Y] > counts[vec3::Z] ? vec3::Y : vec3::Z;
  }

  return best_axis;
}

}

/// Build a sub-tree from an array of leaf nodes.
Node* Tree::BuildSubtree(std::vector<Node*>& leaves, const AABB& aabb,
    unsigned start, unsigned stop, int threaded_depth) {
  if (UNLIKELY((stop - start) == 1)) {
    // Return the leaf node.
    return leaves[start];
  }

  // Determine optimal split direction.
  vec3::Axis axis = BestSplitDirection(leaves, aabb, start, stop);

  // Get the split threshold (2 x middle point along split axis).
  scalar mid2 = aabb.Min()[axis] + aabb.Max()[axis];

  // Sort the leaves array into two new sub-arrays:
  //  first sub-array: start to start_second (exclusive)
  //  second sub-array: start_second to stop (exclusive)
  unsigned start_second = stop;
  for (unsigned i = start; i < start_second;) {
    Node* leaf = leaves[i];

    // Check if this leaf is below or above the split threshold.
    const AABB& leaf_aabb = leaf->BoundingBox();
    scalar leaf_mid2 = leaf_aabb.Min()[axis] + leaf_aabb.Max()[axis];
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

  // Handle degenerate cases (we must have at least one element in each array).
  if (UNLIKELY(start_second == start)) {
    start_second++;
  } else if (UNLIKELY(start_second == stop)) {
    start_second--;
  }

  // Calculate new bounding boxes for the first/second array.
  AABB first_aabb, second_aabb;
  BoundingBoxUnion(first_aabb, leaves, start, start_second);
  BoundingBoxUnion(second_aabb, leaves, start_second, stop);

  // Recursively build new sub-trees.
  Node* first_branch;
  Node* second_branch;
  if (UNLIKELY(threaded_depth >= 1)) {
    // Spawn new threads when we're at the correct depth to better utilize
    // multi-core CPUs.
    std::promise<Node*> result1;
    std::promise<Node*> result2;
    std::thread t1 = std::thread([&] {
        result1.set_value(BuildSubtree(leaves, first_aabb, start, start_second, threaded_depth - 1));
    });
    std::thread t2 = std::thread([&] {
        result2.set_value(BuildSubtree(leaves, second_aabb, start_second, stop, threaded_depth - 1));
    });
    first_branch = result1.get_future().get();
    second_branch = result2.get_future().get();
    t1.join();
    t2.join();
  } else {
    first_branch = BuildSubtree(leaves, first_aabb, start, start_second, threaded_depth - 1);
    second_branch = BuildSubtree(leaves, second_aabb, start_second, stop, threaded_depth - 1);
  }

  // "Create" a new branch node.
  Node* node = &m_branch_nodes[m_branch_nodes_idx++];
  node->Define(aabb, first_branch, second_branch);

  return node;
}

void Tree::Build(std::vector<Node*>& leaves, const AABB& aabb) {
  if (leaves.size() == 0) {
    LOG("Empty tree (no nodes to process).");
    m_root = NULL;
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
  int concurrency = Thread::hardware_concurrency();
  if (concurrency > 8)
    threaded_depth = 4;
  else if (concurrency > 4)
    threaded_depth = 3;
  else if (concurrency > 2)
    threaded_depth = 2;
  else if (concurrency > 1)
    threaded_depth = 1;
  else
    threaded_depth = 0;
  DLOG("Hardware concurrency=%d, threaded_depth=%d", concurrency, threaded_depth);

  // Allocate memory for the branch nodes. We know that we'll get exactly N-1
  // branch nodes (where N is the number of leaf nodes), since we're building a
  // complete binary tree.
  m_branch_nodes.resize(leaves.size() - 1);
  m_branch_nodes_idx = 0;

  // Recursively build the tree, and the result is our root node.
  m_root = BuildSubtree(leaves, aabb, 0, leaves.size(), threaded_depth);
  ASSERT(m_branch_nodes_idx == m_branch_nodes.size(), "Badly built tree!");
}

TriangleTree::TriangleTree(const MeshData* data) {
  ScopedPerf _perf = ScopedPerf("Build triangle tree");

  // We keep a reference to the vertices.
  m_data = data;

  ScopedPerf _leaf_perf = ScopedPerf("Generate leaf nodes");

  // Create a vector of leaf nodes.
  m_leaf_nodes.resize(m_data->triangles.size());
  std::vector<Node*> leaves(m_data->triangles.size());
#ifdef USE_OPENMP
  #pragma omp parallel for
#endif // USE_OPENMP
  for (unsigned i = 0; i < m_data->triangles.size(); i++) {
    // Get the triangle.
    const Triangle* triangle = &m_data->triangles[i];

    // Get the three vertex positions for the triangle.
    const vec3 p1 = m_data->vertices[triangle->a].position;
    const vec3 p2 = m_data->vertices[triangle->b].position;
    const vec3 p3 = m_data->vertices[triangle->c].position;

    // Calculate the bounding box for the triangle.
    AABB triangle_aabb;
    triangle_aabb.Min().x = std::min(p1.x, std::min(p2.x, p3.x));
    triangle_aabb.Min().y = std::min(p1.y, std::min(p2.y, p3.y));
    triangle_aabb.Min().z = std::min(p1.z, std::min(p2.z, p3.z));
    triangle_aabb.Max().x = std::max(p1.x, std::max(p2.x, p3.x));
    triangle_aabb.Max().y = std::max(p1.y, std::max(p2.y, p3.y));
    triangle_aabb.Max().z = std::max(p1.z, std::max(p2.z, p3.z));

    // "Create" a new leaf node.
    TriangleNode* node = static_cast<TriangleNode*>(&m_leaf_nodes[i]);
    node->Define(triangle_aabb, triangle);

    // Add the leaf node to the node pointer vector (for quick sorting).
    leaves[i] = node;
  }

  // Calculate total bounding box.
  AABB aabb;
  BoundingBoxUnion(aabb, leaves, 0, leaves.size());

  _leaf_perf.Done();

  // Build the tree. Here we pass the temporary leaf pointer vector, since
  // it's faster to sort it than the leaf items themselves (less memory to
  // copy).
  Tree::Build(leaves, aabb);

  _perf.Done();
}

bool TriangleTree::Intersect(const Ray& ray, HitInfo& hit) const {
  // Nothing to do?
  if (UNLIKELY(Empty())) {
    return false;
  }

  // Note: Since we usually get here from ObjectTree::RecursiveIntersect(), the
  // root node bounding box of the triangle tree (which equals the object tree
  // leaf node bounding box), has already been checked for intersection.
  // Therefore, we do not have to check intersection against the root node
  // bounding box.

  // Recursively intersect with the entire tree.
  return RecursiveIntersect(m_root, ray, hit);
}

bool TriangleTree::RecursiveIntersect(const Node* node, const Ray& ray,
    HitInfo& hit) const {
  // Was this a leaf node?
  if (node->IsLeafNode()) {
    // Get the triangle of this leaf node.
    const Triangle* triangle =
        reinterpret_cast<const TriangleNode*>(node)->Item();

    // Calculate intersection with the triangle.
    return IntersectTriangle(ray, triangle, hit);
  }

  // Check intersection with children bounding boxes.
  const Node* children[2];
  children[0] = node->FirstChild();
  children[1] = node->SecondChild();
  scalar t1 = hit.t, t2 = hit.t;
  bool got_hit[2];
  got_hit[0] = children[0]->BoundingBox().Intersect(ray, t1);
  got_hit[1] = children[1]->BoundingBox().Intersect(ray, t2);

  // Select optimal recursion based on bounding box intersection results.
  int first = 0, second = 1;
  if (t2 < t1) {
    first = 1;
    second = 0;
  }
  if (got_hit[first]) {
    got_hit[first] = RecursiveIntersect(children[first], ray, hit);
  }
  if (got_hit[second]) {
    got_hit[second] = RecursiveIntersect(children[second], ray, hit);
  }

  return got_hit[0] || got_hit[1];
}

bool TriangleTree::IntersectTriangle(const Ray& ray, const Triangle* triangle,
    HitInfo& hit) const {
  // NOTE: This is an implementation of the Möller / Trumbone fast triangle
  // intersection algorithm.

  // Get the three vertex positions for the triangle.
  const vec3 p1 = m_data->vertices[triangle->a].position;
  const vec3 p2 = m_data->vertices[triangle->b].position;
  const vec3 p3 = m_data->vertices[triangle->c].position;

  // Triangle basis vectors.
  const vec3 e1 = p2 - p1;
  const vec3 e2 = p3 - p1;

  vec3 h = ray.Direction().Cross(e2);
  scalar scale = e1.Dot(h);
  if (scale > -EPSILON && scale < EPSILON) {
    return false;
  }
  scale = scalar(1.0) / scale;

  // Calculate u coordinate of the intersection.
  vec3 s = ray.Origin() - p1;
  scalar u = scale * s.Dot(h);
  if (u < scalar(0.0) || u > scalar(1.0)) {
    return false;
  }

  // Calculate v coordinate of the intersection.
  vec3 q = s.Cross(e1);
  scalar v = scale * ray.Direction().Dot(q);
  if (v < scalar(0.0) || u + v > scalar(1.0)) {
    return false;
  }

  // We have an intersection. Now check if the distance is within the avaliable
  // bounds (0, hit.t).
  scalar t = scale * e2.Dot(q);
  if (t > EPSILON && t < hit.t) {
    // New closest t.
    hit.t = t;

    // We store the triangle and the u/v coordinates for later (e.g. normal
    // interpolation).
    hit.triangle = triangle;
    hit.tri_uv = vec2(u, v);

    return true;
  }

  // No intersection.
  return false;
}

void ObjectTree::Build(const std::list<std::unique_ptr<Object> >& objects) {
  ScopedPerf _perf = ScopedPerf("Build object tree");

  ScopedPerf _leaf_perf = ScopedPerf("Generate leaf nodes");

  // Clear tree.
  m_leaf_nodes.clear();
  m_branch_nodes.clear();

  // Create a vector of leaf nodes, and calculate the total bounding box while
  // we're at it.
  AABB aabb_union;
  m_leaf_nodes.resize(objects.size());
  std::vector<Node*> leaves(objects.size());
  auto it = objects.begin();
  for (unsigned i = 0; i < objects.size(); i++) {
    // Get the object.
    const Object* object = it->get();
    it++;

    // Get the bounding box for the object.
    const AABB aabb = object->TransformedBoundingBox();

    // Update the union bounding box for all the objects.
    if (UNLIKELY(!i)) {
      aabb_union = aabb;
    } else {
      aabb_union += aabb;
    }

    // "Create" a new leaf node.
    ObjectNode* node = static_cast<ObjectNode*>(&m_leaf_nodes[i]);
    node->Define(aabb, object);

    // Add the leaf node to the node pointer vector (for quick sorting).
    leaves[i] = node;
  }

  _leaf_perf.Done();

  // Build the tree. Here we pass the temporary leaf pointer vector, since
  // it's faster to sort it than the leaf items themselves (less memory to
  // copy).
  Tree::Build(leaves, aabb_union);

  _perf.Done();
}

bool ObjectTree::Intersect(const Ray& ray, HitInfo& hit) const {
  // Nothing to do?
  if (UNLIKELY(Empty())) {
    return false;
  }

  // Check intersection against root node bounding box.
  scalar aabb_t = hit.t;
  if (!m_root->BoundingBox().Intersect(ray, aabb_t)) {
    return false;
  }

  // Recursively intersect with the entire tree.
  return RecursiveIntersect(m_root, ray, hit);
}

bool ObjectTree::RecursiveIntersect(const Node* node, const Ray& ray,
   HitInfo& hit) const {
  // Was this a leaf node?
  if (node->IsLeafNode()) {
    // Get the object of this leaf node.
    const Object* object =
        reinterpret_cast<const ObjectNode*>(node)->Item();

    // Calculate intersection with the object.
    return object->Intersect(ray, hit);
  }

  // Check intersection with children bounding boxes.
  const Node* children[2];
  children[0] = node->FirstChild();
  children[1] = node->SecondChild();
  scalar t1 = hit.t, t2 = hit.t;
  bool got_hit[2];
  got_hit[0] = children[0]->BoundingBox().Intersect(ray, t1);
  got_hit[1] = children[1]->BoundingBox().Intersect(ray, t2);

  // Select optimal recursion based on bounding box intersection results.
  int first = 0, second = 1;
  if (t2 < t1) {
    first = 1;
    second = 0;
  }
  if (got_hit[first]) {
    got_hit[first] = RecursiveIntersect(children[first], ray, hit);
  }
  if (got_hit[second]) {
    got_hit[second] = RecursiveIntersect(children[second], ray, hit);
  }

  return got_hit[0] || got_hit[1];
}

} // namespace mageray
