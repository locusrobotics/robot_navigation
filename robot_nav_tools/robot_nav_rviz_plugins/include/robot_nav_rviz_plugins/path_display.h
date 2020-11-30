/*
 * Copyright (c) 2008, Willow Garage, Inc.
 *               2020, Locus Robotics
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
 *     * Neither the name of the Willow Garage, Inc. or Locus Robotics nor the names of its
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

#ifndef ROBOT_NAV_RVIZ_PLUGINS_PATH_DISPLAY_H
#define ROBOT_NAV_RVIZ_PLUGINS_PATH_DISPLAY_H

#include <nav_2d_msgs/Path2D.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <OgreManualObject.h>
#include <vector>

namespace robot_nav_rviz_plugins
{
/**
 * @brief Displays a nav_2d_msgs::Path2D message in Rviz
 *
 * @note this is refactored from rviz/PathDisplay
 */
class PathDisplay: public rviz::MessageFilterDisplay<nav_2d_msgs::Path2D>
{
Q_OBJECT
public:
  PathDisplay();
  virtual ~PathDisplay();
  void reset() override;

protected:
  void onInitialize() override;
  void processMessage(const nav_2d_msgs::Path2D::ConstPtr& msg) override;

private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();
  void updatePoseStyle();
  void updatePoseAxisGeometry();
  void updatePoseArrowColor();
  void updatePoseArrowGeometry();

private:
  // Display Option Enums
  enum struct LineStyle {LINES, BILLBOARDS};
  enum struct PoseStyle {NONE, AXES, ARROWS};
  LineStyle getLineStyle() const { return static_cast<LineStyle>(style_property_->getOptionInt()); }
  PoseStyle getPoseStyle() const { return static_cast<PoseStyle>(pose_style_property_->getOptionInt()); }

  void destroyObjects();
  void allocateArrowVector(std::vector<rviz::Arrow*>& arrow_vect, int num);
  void allocateAxesVector(std::vector<rviz::Axes*>& axes_vect, int num);

  std::vector<Ogre::ManualObject*> manual_objects_;
  std::vector<rviz::BillboardLine*> billboard_lines_;
  std::vector<std::vector<rviz::Axes*>> axes_chain_;
  std::vector<std::vector<rviz::Arrow*>> arrow_chain_;

  rviz::EnumProperty* style_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* line_width_property_;
  rviz::IntProperty* buffer_length_property_;
  rviz::VectorProperty* offset_property_;

  // pose marker property
  rviz::EnumProperty* pose_style_property_;
  rviz::FloatProperty* pose_axes_length_property_;
  rviz::FloatProperty* pose_axes_radius_property_;
  rviz::ColorProperty* pose_arrow_color_property_;
  rviz::FloatProperty* pose_arrow_shaft_length_property_;
  rviz::FloatProperty* pose_arrow_head_length_property_;
  rviz::FloatProperty* pose_arrow_shaft_diameter_property_;
  rviz::FloatProperty* pose_arrow_head_diameter_property_;
};
}  // namespace robot_nav_rviz_plugins

#endif  // ROBOT_NAV_RVIZ_PLUGINS_PATH_DISPLAY_H
