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

#include "overlay_camera_display.h"

#include "overlay_utils.h"

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlayManager.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/validate_floats.hpp>

#include <image_transport/camera_common.hpp>

namespace jsk_rviz_plugins
{
  namespace
  {
    bool validateFloats(const sensor_msgs::msg::CameraInfo& msg)
    {
      return rviz_common::validateFloats(msg.d) &&
        rviz_common::validateFloats(msg.k) &&
        rviz_common::validateFloats(msg.r) &&
        rviz_common::validateFloats(msg.p);
    }
  }

  OverlayCameraDisplay::OverlayCameraDisplay()
    : Display(),
      camera_(nullptr), camera_node_(nullptr),
      bg_scene_node_(nullptr), bg_screen_rect_(nullptr),
      overlay_(nullptr), panel_(nullptr),
      width_(320), height_(240), left_(128), top_(128),
      new_image_arrived_(false)
  {
    update_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Topic", "",
      QString::fromStdString(
        rosidl_generator_traits::name<sensor_msgs::msg::Image>()),
      "sensor_msgs::msg::Image topic to subscribe to. The camera info is "
      "expected on the matching camera_info topic.",
      this, SLOT( updateTopic() ));
    transport_hint_property_ = new ImageTransportHintsProperty(
      "transport hint",
      "transport hint to subscribe topic",
      this, SLOT(updateTopic()));
    width_property_ = new rviz_common::properties::IntProperty(
      "width", 320,
      "width of the overlay",
      this, SLOT(updateWidth()));
    width_property_->setMin(1);
    height_property_ = new rviz_common::properties::IntProperty(
      "height", 240,
      "height of the overlay",
      this, SLOT(updateHeight()));
    height_property_->setMin(1);
    left_property_ = new rviz_common::properties::IntProperty(
      "left", 128,
      "left of the overlay",
      this, SLOT(updateLeft()));
    left_property_->setMin(0);
    top_property_ = new rviz_common::properties::IntProperty(
      "top", 128,
      "top of the overlay",
      this, SLOT(updateTop()));
    top_property_->setMin(0);
    zoom_property_ = new rviz_common::properties::FloatProperty(
      "zoom", 1.0,
      "zoom factor of the camera view",
      this);
    zoom_property_->setMin(0.00001);
    zoom_property_->setMax(100000);
    far_clip_property_ = new rviz_common::properties::FloatProperty(
      "far clip", 100.0,
      "far clip distance of the camera view",
      this);
    far_clip_property_->setMin(0.01);
  }

  OverlayCameraDisplay::~OverlayCameraDisplay()
  {
    unsubscribe();
    destroyRenderTexture();
    if (overlay_) {
      Ogre::OverlayManager* mgr = Ogre::OverlayManager::getSingletonPtr();
      overlay_->hide();
      if (panel_) {
        overlay_->remove2D(panel_);
        mgr->destroyOverlayElement(panel_);
      }
      mgr->destroy(overlay_);
    }
    if (panel_material_) {
      panel_material_->unload();
      Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
    }
    if (bg_screen_rect_) {
      delete bg_screen_rect_;
    }
    if (bg_material_) {
      bg_material_->unload();
      Ogre::MaterialManager::getSingleton().remove(bg_material_->getName());
    }
    if (bg_scene_node_) {
      bg_scene_node_->getParentSceneNode()->removeAndDestroyChild(bg_scene_node_);
    }
  }

  void OverlayCameraDisplay::onInitialize()
  {
    update_topic_property_->initialize(context_->getRosNodeAbstraction());
    prepareOverlays(context_->getSceneManager());

    texture_ = std::make_unique<rviz_default_plugins::displays::ROSImageTexture>();

    static int count = 0;
    rviz_common::UniformStringStream ss;
    ss << "OverlayCameraDisplayObject" << count++;
    const std::string base_name = ss.str();

    // backdrop rectangle showing the camera image, visible only while the
    // offscreen texture is rendered (RenderTargetListener)
    bg_scene_node_ = scene_node_->createChildSceneNode();
    bg_material_ = Ogre::MaterialManager::getSingleton().create(
      base_name + "BgMaterial",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    bg_material_->setDepthWriteEnabled(false);
    bg_material_->setReceiveShadows(false);
    bg_material_->setDepthCheckEnabled(false);
    bg_material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu
      = bg_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    // bind by pointer: the ROSImageTexture texture lives in the
    // "rviz_rendering" resource group and cannot be resolved by name from a
    // material in the default group
    tu->setTexture(texture_->getTexture());
    tu->setTextureFiltering(Ogre::TFO_NONE);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    bg_material_->setCullingMode(Ogre::CULL_NONE);
    bg_material_->setSceneBlending(Ogre::SBT_REPLACE);

    bg_screen_rect_ = new Ogre::Rectangle2D(true);
    bg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
    bg_screen_rect_->setUVs(Ogre::Vector2(0.0f, 0.0f), Ogre::Vector2(0.0f, 1.0f),
                            Ogre::Vector2(1.0f, 0.0f), Ogre::Vector2(1.0f, 1.0f));
    bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    bg_screen_rect_->setBoundingBox(aabInf);
    bg_screen_rect_->setMaterial(bg_material_);
    bg_scene_node_->attachObject(bg_screen_rect_);
    bg_scene_node_->setVisible(false);

    // offscreen camera
    camera_ = context_->getSceneManager()->createCamera(base_name + "Camera");
    camera_node_ = scene_node_->createChildSceneNode();
    camera_node_->attachObject(camera_);
    camera_->setNearClipDistance(0.01f);

    // overlay showing the render texture
    panel_material_ = Ogre::MaterialManager::getSingleton().create(
      base_name + "Material",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::OverlayManager* mgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mgr->create(base_name);
    panel_ = static_cast<Ogre::PanelOverlayElement*>(
      mgr->createOverlayElement("Panel", base_name + "Panel"));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    panel_->setUV(0.0, 0.0, 1.0, 1.0);
    panel_->setMaterialName(panel_material_->getName());
    overlay_->add2D(panel_);

    updateWidth();
    updateHeight();
    updateLeft();
    updateTop();
    updateTopic();
  }

  void OverlayCameraDisplay::ensureRenderTexture(unsigned int width,
                                                 unsigned int height)
  {
    if (render_texture_ &&
        render_texture_->getWidth() == width &&
        render_texture_->getHeight() == height) {
      return;
    }
    destroyRenderTexture();
    render_texture_ = Ogre::TextureManager::getSingleton().createManual(
      panel_material_->getName() + "Texture",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, width, height, 0,
      Ogre::PF_A8R8G8B8, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* target
      = render_texture_->getBuffer()->getRenderTarget();
    Ogre::Viewport* vp = target->addViewport(camera_);
    vp->setClearEveryFrame(true);
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0, 0));
    vp->setOverlaysEnabled(false);
    vp->setShadowsEnabled(false);
    target->addListener(this);
    target->setAutoUpdated(true);
    target->setActive(true);

    panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    panel_material_->getTechnique(0)->getPass(0)
      ->createTextureUnitState(render_texture_->getName());
    panel_material_->getTechnique(0)->getPass(0)
      ->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }

  void OverlayCameraDisplay::destroyRenderTexture()
  {
    if (render_texture_) {
      Ogre::RenderTexture* target
        = render_texture_->getBuffer()->getRenderTarget();
      target->removeListener(this);
      target->removeAllViewports();
      Ogre::TextureManager::getSingleton().remove(render_texture_->getName());
      render_texture_.reset();
    }
  }

  void OverlayCameraDisplay::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent& /*evt*/)
  {
    if (bg_scene_node_ && current_image_) {
      bg_scene_node_->setVisible(true);
    }
  }

  void OverlayCameraDisplay::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent& /*evt*/)
  {
    if (bg_scene_node_) {
      bg_scene_node_->setVisible(false);
    }
  }

  void OverlayCameraDisplay::onEnable()
  {
    subscribe();
    if (overlay_) {
      overlay_->show();
    }
    if (render_texture_) {
      render_texture_->getBuffer()->getRenderTarget()->setActive(true);
    }
  }

  void OverlayCameraDisplay::onDisable()
  {
    unsubscribe();
    if (overlay_) {
      overlay_->hide();
    }
    if (render_texture_) {
      render_texture_->getBuffer()->getRenderTarget()->setActive(false);
    }
  }

  void OverlayCameraDisplay::subscribe()
  {
    if (!isEnabled()) {
      return;
    }
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      rclcpp::Node::SharedPtr node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
      image_sub_ = image_transport::create_subscription(
        node.get(), topic_name,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
          processImage(msg);
        },
        transport_hint_property_->getTransportHints(),
        rmw_qos_profile_sensor_data);
      std::string caminfo_topic
        = image_transport::getCameraInfoTopic(topic_name);
      caminfo_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        caminfo_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          current_caminfo_ = msg;
        });
    }
  }

  void OverlayCameraDisplay::unsubscribe()
  {
    image_sub_.shutdown();
    caminfo_sub_.reset();
  }

  void OverlayCameraDisplay::processImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_image_ = msg;
    texture_->addMessage(msg);
    new_image_arrived_ = true;
  }

  bool OverlayCameraDisplay::updateCamera()
  {
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    sensor_msgs::msg::Image::ConstSharedPtr image;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      info = current_caminfo_;
      image = current_image_;
    }
    if (!image) {
      return false;
    }
    if (!info) {
      setStatus(rviz_common::properties::StatusProperty::Warn,
                "Camera Info", "No CameraInfo received yet");
      return false;
    }
    if (!validateFloats(*info)) {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Camera Info",
                "Contains invalid floating point values (nans or infs)");
      return false;
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    rclcpp::Time time_stamp(image->header.stamp, RCL_ROS_TIME);
    if (!context_->getFrameManager()->getTransform(
          image->header.frame_id, time_stamp, position, orientation)) {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Transform",
                QString("Could not transform from [")
                + image->header.frame_id.c_str() + "] to fixed frame");
      return false;
    }
    setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "OK");

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    orientation = orientation
      * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

    double img_width = info->width;
    double img_height = info->height;
    if (img_width == 0) {
      img_width = image->width;
    }
    if (img_height == 0) {
      img_height = image->height;
    }
    if (img_width == 0 || img_height == 0) {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Camera Info", "Could not determine image width/height");
      return false;
    }

    double fx = info->p[0];
    double fy = info->p[5];
    if (fx == 0.0 || fy == 0.0) {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Camera Info", "Invalid intrinsic matrix (fx or fy is 0)");
      return false;
    }

    // add the camera's translation relative to the left camera (from P[3])
    double tx = -1.0 * (info->p[3] / fx);
    Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
    position = position + (right * tx);
    double ty = -1.0 * (info->p[7] / fy);
    Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
    position = position + (down * ty);

    if (!rviz_common::validateFloats(position)) {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Camera Info",
                "CameraInfo/P resulted in an invalid position");
      return false;
    }

    camera_node_->setPosition(position);
    camera_node_->setOrientation(orientation);

    // preserve the image aspect ratio in the overlay viewport
    float zoom_x = zoom_property_->getFloat();
    float zoom_y = zoom_x;
    float win_width = width_;
    float win_height = height_;
    if (win_width != 0 && win_height != 0) {
      float img_aspect = (img_width / fx) / (img_height / fy);
      float win_aspect = win_width / win_height;
      if (img_aspect > win_aspect) {
        zoom_y = zoom_y / img_aspect * win_aspect;
      }
      else {
        zoom_x = zoom_x / win_aspect * img_aspect;
      }
    }

    double cx = info->p[2];
    double cy = info->p[6];
    float far_plane = far_clip_property_->getFloat();
    float near_plane = 0.01f;
    Ogre::Matrix4 proj_matrix = Ogre::Matrix4::ZERO;
    proj_matrix[0][0] = 2.0f * fx / img_width * zoom_x;
    proj_matrix[1][1] = 2.0f * fy / img_height * zoom_y;
    proj_matrix[0][2] = 2.0f * (0.5f - cx / img_width) * zoom_x;
    proj_matrix[1][2] = 2.0f * (cy / img_height - 0.5f) * zoom_y;
    proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
    proj_matrix[2][3] = -2.0f * far_plane * near_plane / (far_plane - near_plane);
    proj_matrix[3][2] = -1;
    camera_->setCustomProjectionMatrix(true, proj_matrix);

    setStatus(rviz_common::properties::StatusProperty::Ok,
              "Camera Info", "OK");

    // scale the backdrop so the image aspect matches the viewport
    // (identity: the rectangle fills the viewport; the aspect correction is
    // already folded into the projection matrix through zoom_x/zoom_y)
    bg_screen_rect_->setCorners(
      -1.0f * zoom_x / zoom_property_->getFloat(),
      1.0f * zoom_y / zoom_property_->getFloat(),
      1.0f * zoom_x / zoom_property_->getFloat(),
      -1.0f * zoom_y / zoom_property_->getFloat());
    return true;
  }

  void OverlayCameraDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
  {
    if (!isEnabled()) {
      return;
    }
    if (new_image_arrived_) {
      try {
        texture_->update();
      }
      catch (std::exception& e) {
        setStatus(rviz_common::properties::StatusProperty::Error,
                  "Image", QString("Could not convert image: ") + e.what());
        return;
      }
      new_image_arrived_ = false;
    }
    ensureRenderTexture(width_, height_);
    if (!updateCamera()) {
      return;
    }
    panel_->setPosition(left_, top_);
    panel_->setDimensions(width_, height_);
    if (!overlay_->isVisible()) {
      overlay_->show();
    }
    context_->queueRender();
  }

  void OverlayCameraDisplay::reset()
  {
    Display::reset();
    std::lock_guard<std::mutex> lock(mutex_);
    current_caminfo_.reset();
    current_image_.reset();
  }

  void OverlayCameraDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }

  void OverlayCameraDisplay::updateWidth()
  {
    width_ = width_property_->getInt();
  }

  void OverlayCameraDisplay::updateHeight()
  {
    height_ = height_property_->getInt();
  }

  void OverlayCameraDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
  }

  void OverlayCameraDisplay::updateTop()
  {
    top_ = top_property_->getInt();
  }

  bool OverlayCameraDisplay::isInRegion(int x, int y)
  {
    return (top_ < y && top_ + height_ > y &&
            left_ < x && left_ + width_ > x);
  }

  void OverlayCameraDisplay::movePosition(int x, int y)
  {
    top_ = y;
    left_ = x;
  }

  void OverlayCameraDisplay::setPosition(int x, int y)
  {
    top_property_->setValue(y);
    left_property_->setValue(x);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::OverlayCameraDisplay, rviz_common::Display)
