// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Iori Yanokura
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

#include "rviz_scene_publisher.h"
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include "render_window_compat.h"
#include <OgreRenderTarget.h>
#include <OgreViewport.h>

namespace jsk_rviz_plugins
{
  RvizScenePublisher::RvizScenePublisher():
    Display(), image_id_(0)
  {
    topic_name_property_ = new rviz_common::properties::StringProperty(
      "topic_name", "/rviz/image",
      "topic_name", this, SLOT(updateTopicName()));
  }

  RvizScenePublisher::~RvizScenePublisher()
  {
    delete topic_name_property_;
  }

  void RvizScenePublisher::onInitialize()
  {
    updateTopicName();
    context_->queueRender();
  }

  void RvizScenePublisher::onEnable()
  {
    context_->queueRender();
  }

  void RvizScenePublisher::updateTopicName()
  {
    if (!context_) {
      return;
    }
    auto node_abstraction = context_->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
      return;
    }
    topic_name_ = topic_name_property_->getStdString();
    publisher_ = image_transport::create_publisher(
      node_abstraction->get_raw_node().get(), topic_name_);
  }

  void RvizScenePublisher::update(float /*wall_dt*/, float /*ros_dt*/)
  {
    // read the pixels straight out of the Ogre render target; Qt window
    // grabbing breaks the GL compositing of the render panel
    rviz_rendering::RenderWindow* render_window
      = context_->getViewManager()->getRenderPanel()->getRenderWindow();
    Ogre::RenderTarget* target
      = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window)
        ->getTarget();
    const unsigned int w = target->getWidth();
    const unsigned int h = target->getHeight();
    if (w == 0 || h == 0) {
      return;
    }
    cv::Mat image(h, w, CV_8UC3);  // RGB
    Ogre::PixelBox pixel_box(w, h, 1, Ogre::PF_BYTE_RGB, image.data);
    target->copyContentsToMemory(Ogre::Box(0, 0, w, h), pixel_box,
                                 Ogre::RenderTarget::FB_AUTO);

    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    header.stamp = context_->getClock()->now();
    image_id_++;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    img_bridge.toImageMsg(img_msg);
    publisher_.publish(img_msg);
  }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::RvizScenePublisher, rviz_common::Display)
