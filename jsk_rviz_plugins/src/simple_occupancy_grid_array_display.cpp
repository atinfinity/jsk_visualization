// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
#include "simple_occupancy_grid_array_display.h"
#include <Eigen/Geometry>
#include <jsk_topic_tools/color_utils.h>
#include <jsk_recognition_utils/geo/plane.h>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>

namespace jsk_rviz_plugins
{
  SimpleOccupancyGridArrayDisplay::SimpleOccupancyGridArrayDisplay()
  {
    auto_color_property_ = new rviz_common::properties::BoolProperty(
      "Auto Color", true,
      "Auto coloring",
      this, SLOT(updateAutoColor()));
    
    alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 1.0,
      "Amount of transparency to apply to the polygon.",
      this, SLOT(updateAlpha()));

    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    
  }

  SimpleOccupancyGridArrayDisplay::~SimpleOccupancyGridArrayDisplay()
  {
    delete alpha_property_;
    allocateCloudsAndNodes(0);
  }

  void SimpleOccupancyGridArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    updateAlpha();
    updateAutoColor();
  }
  
  void SimpleOccupancyGridArrayDisplay::allocateCloudsAndNodes(const size_t num)
  {
    if (num > clouds_.size()) { // need to allocate new node and clouds
      for (size_t i = clouds_.size(); i < num; i++) {
        Ogre::SceneNode* node = scene_node_->createChildSceneNode();
        rviz_rendering::PointCloud* cloud = new rviz_rendering::PointCloud();
        cloud->setRenderMode(rviz_rendering::PointCloud::RM_TILES);
        cloud->setCommonDirection( Ogre::Vector3::UNIT_Z );
        cloud->setCommonUpVector( Ogre::Vector3::UNIT_Y );
        node->attachObject(cloud);
        clouds_.push_back(cloud);
        nodes_.push_back(node);
      }
    }
    else if (num < clouds_.size()) // need to destroy
    {
      for (size_t i = num; i < clouds_.size(); i++) {
        nodes_[i]->detachObject(clouds_[i]);
        delete clouds_[i];
        scene_manager_->destroySceneNode(nodes_[i]);
      }
      clouds_.resize(num);
      nodes_.resize(num);
    }
  }

  void SimpleOccupancyGridArrayDisplay::reset()
  {
    MFDClass::reset();
    allocateCloudsAndNodes(0);
  }

  void SimpleOccupancyGridArrayDisplay::processMessage(
    jsk_recognition_msgs::msg::SimpleOccupancyGridArray::ConstSharedPtr msg)
  {
    Ogre::ColourValue white(1, 1, 1, 1);
    allocateCloudsAndNodes(msg->grids.size()); // not enough
    for (size_t i = 0; i < msg->grids.size(); i++) {
      Ogre::SceneNode* node = nodes_[i];
      rviz_rendering::PointCloud* cloud = clouds_[i];
      const jsk_recognition_msgs::msg::SimpleOccupancyGrid grid = msg->grids[i];
      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      
      // coefficients
      geometry_msgs::msg::Pose plane_pose;
      jsk_recognition_utils::Plane::Ptr plane(new jsk_recognition_utils::Plane(
        std::vector<float>(grid.coefficients.begin(), grid.coefficients.end())));
      Eigen::Affine3f plane_pose_eigen = plane->coordinates();
      Eigen::Vector3f plane_translation(plane_pose_eigen.translation());
      Eigen::Quaternionf plane_rotation(plane_pose_eigen.rotation());
      plane_pose.position.x = plane_translation[0];
      plane_pose.position.y = plane_translation[1];
      plane_pose.position.z = plane_translation[2];
      plane_pose.orientation.x = plane_rotation.x();
      plane_pose.orientation.y = plane_rotation.y();
      plane_pose.orientation.z = plane_rotation.z();
      plane_pose.orientation.w = plane_rotation.w();
      if(!context_->getFrameManager()->transform(grid.header, plane_pose,
                                                 position,
                                                 quaternion)) {
        std::ostringstream oss;
        oss << "Error transforming pose";
        oss << " from frame '" << grid.header.frame_id << "'";
        oss << " to frame '" << qPrintable(fixed_frame_) << "'";
        RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
        setStatus(rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
        return;
      }
      node->setPosition(position);
      node->setOrientation(quaternion);
      cloud->setDimensions(grid.resolution, grid.resolution, 0.0);
      std::vector<rviz_rendering::PointCloud::Point> points;
      for (size_t ci = 0; ci < grid.cells.size(); ci++) {
        const geometry_msgs::msg::Point p = grid.cells[ci];
        rviz_rendering::PointCloud::Point point;
        if (auto_color_) {
          std_msgs::msg::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(i);
          point.color = Ogre::ColourValue(ros_color.r, ros_color.g, ros_color.b, ros_color.a);
        }
        else {
          point.color = white;
        }
        point.position.x = p.x;
        point.position.y = p.y;
        point.position.z = p.z;
        points.push_back(point);
      }
      cloud->clear();
      cloud->setAlpha(alpha_);
      if (!points.empty()) {
        cloud->addPoints(points.begin(), points.end());
      }
    }
    context_->queueRender();
  }
  
  void SimpleOccupancyGridArrayDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
  }

  void SimpleOccupancyGridArrayDisplay::updateAutoColor()
  {
    auto_color_ = auto_color_property_->getBool();
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::SimpleOccupancyGridArrayDisplay, rviz_common::Display )

