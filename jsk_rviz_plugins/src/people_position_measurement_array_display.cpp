// -*- mode: c++ -*-
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

#include "people_position_measurement_array_display.h"
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/logging.hpp>
#include <OgreCamera.h>
#include <QPainter>
#include <rviz_rendering/render_system.hpp>
#include <OgreRenderSystem.h>

#include <algorithm>

namespace jsk_rviz_plugins
{
  PeoplePositionMeasurementArrayDisplay::PeoplePositionMeasurementArrayDisplay()
  {
    size_property_ = new rviz_common::properties::FloatProperty(
      "size", 0.3,
      "size of the visualizer", this,
      SLOT(updateSize()));
    timeout_property_ = new rviz_common::properties::FloatProperty(
      "timeout", 10.0, "timeout seconds", this, SLOT(updateTimeout()));
    anonymous_property_ = new rviz_common::properties::BoolProperty(
      "anonymous", false,
      "anonymous",
      this, SLOT(updateAnonymous()));
    text_property_ = new rviz_common::properties::StringProperty(
      "text", "person found here person found here",
      "text to rotate",
      this, SLOT(updateText()));
  }

  
  PeoplePositionMeasurementArrayDisplay::~PeoplePositionMeasurementArrayDisplay()
  {
    delete size_property_;
  }

  void PeoplePositionMeasurementArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    updateSize();
    updateTimeout();
    updateAnonymous();
    updateText();
  }

  void PeoplePositionMeasurementArrayDisplay::clearObjects()
  {
    faces_.clear();
    visualizers_.clear();
  }
  
  void PeoplePositionMeasurementArrayDisplay::reset()
  {
    MFDClass::reset();
    clearObjects();
  }

  void PeoplePositionMeasurementArrayDisplay::processMessage(
    people_msgs::msg::PositionMeasurementArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    static int count = 0;
    static int square_count = 0;
    faces_ = msg->people;
    // check texture is ready or not
    if (faces_.size() > visualizers_.size()) {
      for (size_t i = visualizers_.size(); i < faces_.size(); i++) {
        visualizers_.push_back(GISCircleVisualizer::Ptr(new GISCircleVisualizer(
                                                          scene_manager_,
                                                          scene_node_,
                                                          size_,
                                                          text_)));
        visualizers_[visualizers_.size() - 1]->setAnonymous(anonymous_);
        visualizers_[visualizers_.size() - 1]->update(0, 0);
        QColor color(25.0, 255.0, 240.0);
        visualizers_[visualizers_.size() - 1]->setColor(color);
      }
    }
    else {
      visualizers_.resize(faces_.size());
    }
    // move scene_node
    for (size_t i = 0; i < faces_.size(); i++) {
      Ogre::Quaternion orientation;
      Ogre::Vector3 position;
      geometry_msgs::msg::Pose pose;
      pose.position = faces_[i].pos;
      pose.orientation.w = 1.0;
      if(!context_->getFrameManager()->transform(msg->header,
                                                 pose,
                                                 position, orientation))
      {
        std::ostringstream oss;
        oss << "Error transforming pose";
        oss << " from frame '" << msg->header.frame_id << "'";
        oss << " to frame '" << qPrintable(fixed_frame_) << "'";
        RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
        setStatus(rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
      }
      else {
        visualizers_[i]->setPosition(position);
      }
    }
    latest_time_ = rclcpp::Time(msg->header.stamp,
                                context_->getClock()->get_clock_type());
  }

  void PeoplePositionMeasurementArrayDisplay::update(
    float wall_dt, float ros_dt)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (faces_.size() == 0) {
      return;
    }
    if ((context_->getClock()->now() - latest_time_).seconds() > timeout_) {
      RVIZ_COMMON_LOG_WARNING("timeout face recognition result");
      clearObjects();
      return;
    }
    for (size_t i = 0; i < visualizers_.size(); i++) {
      visualizers_[i]->setOrientation(context_);
    }
    for (size_t i = 0; i < visualizers_.size(); i++) {
      visualizers_[i]->update(wall_dt, ros_dt);
    }
  }

  void PeoplePositionMeasurementArrayDisplay::updateTimeout()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_ = timeout_property_->getFloat();
  }
  
  void PeoplePositionMeasurementArrayDisplay::updateSize()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_ = size_property_->getFloat();
    visualizers_.clear();
  }

  void PeoplePositionMeasurementArrayDisplay::updateAnonymous()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    anonymous_ = anonymous_property_->getBool();
    for (size_t i = 0; i < visualizers_.size(); i++) {
      visualizers_[i]->setAnonymous(anonymous_);
    }
  }

  void PeoplePositionMeasurementArrayDisplay::updateText()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    text_ = text_property_->getStdString();
    for (size_t i = 0; i < visualizers_.size(); i++) {
      visualizers_[i]->setText(text_);
    }
  }
  
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::PeoplePositionMeasurementArrayDisplay, rviz_common::Display )

