// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Vendored, PCL-free subset of jsk_recognition_utils::Polygon
// (https://github.com/jsk-ros-pkg/jsk_recognition) carrying only the API
// used by jsk_rviz_plugins. decomposeToTriangles() uses a plain
// ear-clipping triangulation instead of the PCL-based one. Remove once
// jsk_recognition_utils is released for ROS 2.

#ifndef JSK_RECOGNITION_UTILS_GEO_POLYGON_H_
#define JSK_RECOGNITION_UTILS_GEO_POLYGON_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <geometry_msgs/msg/polygon.hpp>

#include <memory>
#include <vector>

#include "jsk_recognition_utils/geo/plane.h"

namespace jsk_recognition_utils
{
  typedef std::vector<Eigen::Vector3f,
                      Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;

  class Polygon: public Plane
  {
  public:
    typedef std::shared_ptr<Polygon> Ptr;

    Polygon(const Vertices& vertices)
      : Plane(normalFromVertices(vertices),
              vertices.empty() ? Eigen::Vector3f::Zero() : vertices[0]),
        vertices_(vertices)
    {
    }

    static Polygon fromROSMsg(const geometry_msgs::msg::Polygon& polygon)
    {
      Vertices vertices;
      for (size_t i = 0; i < polygon.points.size(); i++) {
        vertices.push_back(Eigen::Vector3f(polygon.points[i].x,
                                           polygon.points[i].y,
                                           polygon.points[i].z));
      }
      return Polygon(vertices);
    }

    size_t getNumVertices() { return vertices_.size(); }
    Eigen::Vector3f getVertex(size_t i) { return vertices_[i]; }
    Vertices getVertices() { return vertices_; }

    Eigen::Vector3f centroid()
    {
      Eigen::Vector3f c(0, 0, 0);
      if (vertices_.size() == 0) {
        return c;
      }
      for (size_t i = 0; i < vertices_.size(); i++) {
        c = c + vertices_[i];
      }
      return c / vertices_.size();
    }

    // Ear-clipping triangulation on the polygon plane.
    std::vector<Polygon::Ptr> decomposeToTriangles()
    {
      std::vector<Polygon::Ptr> triangles;
      if (vertices_.size() < 3) {
        return triangles;
      }
      if (vertices_.size() == 3) {
        triangles.push_back(Polygon::Ptr(new Polygon(vertices_)));
        return triangles;
      }

      // project vertices onto the plane to work in 2D
      Eigen::Affine3f inv = coordinates().inverse();
      std::vector<Eigen::Vector2f,
                  Eigen::aligned_allocator<Eigen::Vector2f> > points2d;
      for (size_t i = 0; i < vertices_.size(); i++) {
        Eigen::Vector3f p = inv * vertices_[i];
        points2d.push_back(Eigen::Vector2f(p[0], p[1]));
      }

      std::vector<size_t> indices(vertices_.size());
      for (size_t i = 0; i < indices.size(); i++) {
        indices[i] = i;
      }

      // polygon orientation from the signed area
      double signed_area = 0.0;
      for (size_t i = 0; i < points2d.size(); i++) {
        const Eigen::Vector2f& a = points2d[i];
        const Eigen::Vector2f& b = points2d[(i + 1) % points2d.size()];
        signed_area += a[0] * b[1] - b[0] * a[1];
      }
      const double orientation = signed_area >= 0.0 ? 1.0 : -1.0;

      size_t guard = 0;
      const size_t max_iterations = indices.size() * indices.size() + 10;
      while (indices.size() > 3 && guard++ < max_iterations) {
        bool ear_found = false;
        for (size_t i = 0; i < indices.size(); i++) {
          const size_t prev = indices[(i + indices.size() - 1) % indices.size()];
          const size_t curr = indices[i];
          const size_t next = indices[(i + 1) % indices.size()];
          const Eigen::Vector2f& a = points2d[prev];
          const Eigen::Vector2f& b = points2d[curr];
          const Eigen::Vector2f& c = points2d[next];
          const double cross = (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0]);
          if (cross * orientation <= 0.0) { // reflex vertex
            continue;
          }
          bool contains_other = false;
          for (size_t j = 0; j < indices.size(); j++) {
            const size_t idx = indices[j];
            if (idx == prev || idx == curr || idx == next) {
              continue;
            }
            if (pointInTriangle(points2d[idx], a, b, c, orientation)) {
              contains_other = true;
              break;
            }
          }
          if (contains_other) {
            continue;
          }
          Vertices tri;
          tri.push_back(vertices_[prev]);
          tri.push_back(vertices_[curr]);
          tri.push_back(vertices_[next]);
          triangles.push_back(Polygon::Ptr(new Polygon(tri)));
          indices.erase(indices.begin() + i);
          ear_found = true;
          break;
        }
        if (!ear_found) {       // degenerate polygon; fall back to a fan
          break;
        }
      }
      if (indices.size() == 3) {
        Vertices tri;
        tri.push_back(vertices_[indices[0]]);
        tri.push_back(vertices_[indices[1]]);
        tri.push_back(vertices_[indices[2]]);
        triangles.push_back(Polygon::Ptr(new Polygon(tri)));
      }
      else if (indices.size() > 3) { // fan fallback for degenerate input
        for (size_t i = 1; i + 1 < indices.size(); i++) {
          Vertices tri;
          tri.push_back(vertices_[indices[0]]);
          tri.push_back(vertices_[indices[i]]);
          tri.push_back(vertices_[indices[i + 1]]);
          triangles.push_back(Polygon::Ptr(new Polygon(tri)));
        }
      }
      return triangles;
    }

  protected:
    static Eigen::Vector3f normalFromVertices(const Vertices& vertices)
    {
      if (vertices.size() >= 3) {
        Eigen::Vector3f n
          = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        if (n.norm() > 0.0) {
          return n.normalized();
        }
      }
      return Eigen::Vector3f::UnitZ();
    }

    static bool pointInTriangle(const Eigen::Vector2f& p,
                                const Eigen::Vector2f& a,
                                const Eigen::Vector2f& b,
                                const Eigen::Vector2f& c,
                                double orientation)
    {
      const double d1 = ((b[0] - a[0]) * (p[1] - a[1])
                         - (b[1] - a[1]) * (p[0] - a[0])) * orientation;
      const double d2 = ((c[0] - b[0]) * (p[1] - b[1])
                         - (c[1] - b[1]) * (p[0] - b[0])) * orientation;
      const double d3 = ((a[0] - c[0]) * (p[1] - c[1])
                         - (a[1] - c[1]) * (p[0] - c[0])) * orientation;
      return d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0;
    }

    Vertices vertices_;
  };
}

#endif
