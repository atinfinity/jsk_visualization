/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// author: Adam Leeper

#ifndef _MARKER_HELPERS_H_
#define _MARKER_HELPERS_H_

#include <interactive_markers/tools.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

namespace im_helpers {

enum PoseState {UNTESTED, VALID, INVALID};

visualization_msgs::msg::InteractiveMarker makeEmptyMarker( const char *frame_id = "" );

visualization_msgs::msg::Marker makeBox( float scale );

visualization_msgs::msg::Marker makeSphere( float scale );
void add3Dof2DControl( visualization_msgs::msg::InteractiveMarker &msg, bool fixed = false);
void add6DofControl( visualization_msgs::msg::InteractiveMarker &msg, bool fixed = false );
void addVisible6DofControl( visualization_msgs::msg::InteractiveMarker &msg, bool fixed = false, bool visible = true );


visualization_msgs::msg::InteractiveMarkerControl& makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg );

visualization_msgs::msg::InteractiveMarkerControl& makeSphereControl( visualization_msgs::msg::InteractiveMarker &msg );

visualization_msgs::msg::MenuEntry makeMenuEntry(const char *title);

visualization_msgs::msg::MenuEntry makeMenuEntry(const char *title, const char *command, int type  );

visualization_msgs::msg::InteractiveMarker makePostureMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped, 
                                                         float scale, bool fixed, bool view_facing );

visualization_msgs::msg::InteractiveMarker makeHeadGoalMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped, 
                                                          float scale);

visualization_msgs::msg::InteractiveMarker makeMeshMarker( const std::string &name, const std::string &mesh_resource,
                                                      const geometry_msgs::msg::PoseStamped &stamped, float scale );

visualization_msgs::msg::InteractiveMarker makeMeshMarker( const std::string &name, const std::string &mesh_resource,
                                                      const geometry_msgs::msg::PoseStamped &stamped, float scale, const std_msgs::msg::ColorRGBA &color );

visualization_msgs::msg::InteractiveMarker makeMeshMarker( const std::string &name, const std::string &mesh_resource,
                                                      const geometry_msgs::msg::PoseStamped &stamped, float scale, const std_msgs::msg::ColorRGBA &color, bool use_color );

visualization_msgs::msg::InteractiveMarker makeButtonBox( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                     float scale, bool fixed, bool view_facing );

visualization_msgs::msg::InteractiveMarker makeButtonSphere( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                     float scale, bool fixed, bool view_facing );

visualization_msgs::msg::InteractiveMarker makeButtonSphere( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                     float scale, bool fixed, bool view_facing, std_msgs::msg::ColorRGBA color );

visualization_msgs::msg::InteractiveMarker makeListControl( const char *name, const geometry_msgs::msg::PoseStamped &stamped, int num, int total, float scale);

visualization_msgs::msg::InteractiveMarker make6DofMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                      float scale, bool fixed, bool view_facing );

visualization_msgs::msg::InteractiveMarker makePlanarMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                      float scale, bool fixed );

visualization_msgs::msg::InteractiveMarker makeElevatorMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                          float scale, bool fixed);

visualization_msgs::msg::InteractiveMarker makeProjectorMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                           float scale);


visualization_msgs::msg::InteractiveMarker makeBaseMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                      float scale, bool fixed);

visualization_msgs::msg::InteractiveMarker makeGripperMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                         float scale, float angle, bool view_facing );

visualization_msgs::msg::InteractiveMarker makeGripperMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                         float scale, float angle, bool view_facing, std_msgs::msg::ColorRGBA color );

visualization_msgs::msg::InteractiveMarker makeGripperMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped,
                                                         float scale, float angle, bool view_facing, std_msgs::msg::ColorRGBA color, bool use_color );

visualization_msgs::msg::InteractiveMarker makeGraspMarker( const char * name, const geometry_msgs::msg::PoseStamped &stamped, float scale, PoseState pose_state);

visualization_msgs::msg::InteractiveMarker makePosedMultiMeshMarker( const char * name, const geometry_msgs::msg::PoseStamped &stamped,
                                                            const std::vector< geometry_msgs::msg::PoseStamped> &mesh_poses,
                                                            const std::vector<std::string> &mesh_paths, const float &scale, const bool button_only = true);

visualization_msgs::msg::InteractiveMarker makeFollowerMultiMeshMarker( const char * name, const geometry_msgs::msg::PoseStamped &stamped,
                                                                   const std::vector<std::string> &mesh_frames,
                                                                   const std::vector<std::string> &mesh_paths,
                                                                   const float &scale);



} // namespace im_helpers

#endif
