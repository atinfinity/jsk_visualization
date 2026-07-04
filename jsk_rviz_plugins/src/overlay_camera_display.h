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

// ROS 2 rewrite: instead of forking the rviz camera display internals,
// this renders the 3D scene from the camera viewpoint into an offscreen
// Ogre render texture (with the live image as backdrop) and shows that
// texture as a 2D overlay on the main rviz view.

#ifndef JSK_RVIZ_PLUGIN_OVERLAY_CAMERA_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_CAMERA_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/image/ros_image_texture.hpp>

#include <OgreCamera.h>
#include <OgreMaterial.h>
#include <OgreRectangle2D.h>
#include <OgreRenderTargetListener.h>
#include <OgreSceneNode.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <mutex>

#include "image_transport_hints_property.h"
#endif

namespace jsk_rviz_plugins
{
  class OverlayCameraDisplay: public rviz_common::Display,
                              public Ogre::RenderTargetListener
  {
    Q_OBJECT
  public:
    OverlayCameraDisplay();
    virtual ~OverlayCameraDisplay();

    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; }
    virtual int getY() { return top_; }

    // Ogre::RenderTargetListener: show the image backdrop only while the
    // offscreen texture is being rendered.
    void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) override;
    void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) override;

  protected:
    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

    virtual void subscribe();
    virtual void unsubscribe();
    virtual void processImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    virtual void ensureRenderTexture(unsigned int width, unsigned int height);
    virtual void destroyRenderTexture();
    virtual bool updateCamera();

    ////////////////////////////////////////////////////////
    // properties
    ////////////////////////////////////////////////////////
    rviz_common::properties::RosTopicProperty* update_topic_property_;
    ImageTransportHintsProperty* transport_hint_property_;
    rviz_common::properties::IntProperty* width_property_;
    rviz_common::properties::IntProperty* height_property_;
    rviz_common::properties::IntProperty* left_property_;
    rviz_common::properties::IntProperty* top_property_;
    rviz_common::properties::FloatProperty* zoom_property_;
    rviz_common::properties::FloatProperty* far_clip_property_;

    ////////////////////////////////////////////////////////
    // ROS
    ////////////////////////////////////////////////////////
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr current_caminfo_;
    sensor_msgs::msg::Image::ConstSharedPtr current_image_;
    std::mutex mutex_;

    ////////////////////////////////////////////////////////
    // Ogre
    ////////////////////////////////////////////////////////
    std::unique_ptr<rviz_default_plugins::displays::ROSImageTexture> texture_;
    Ogre::TexturePtr render_texture_;
    Ogre::Camera* camera_;
    Ogre::SceneNode* camera_node_;
    Ogre::SceneNode* bg_scene_node_;
    Ogre::Rectangle2D* bg_screen_rect_;
    Ogre::MaterialPtr bg_material_;
    Ogre::Overlay* overlay_;
    Ogre::PanelOverlayElement* panel_;
    Ogre::MaterialPtr panel_material_;

    int width_;
    int height_;
    int left_;
    int top_;
    bool new_image_arrived_;

  protected Q_SLOTS:
    void updateTopic();
    void updateWidth();
    void updateHeight();
    void updateLeft();
    void updateTop();
  };
}

#endif
