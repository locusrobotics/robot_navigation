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

#include <robot_nav_rviz_plugins/path_display.h>
#include <robot_nav_rviz_plugins/validate_floats.h>
#include <nav_2d_utils/conversions.h>
#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMatrix4.h>

#include <algorithm>
#include <vector>

namespace robot_nav_rviz_plugins
{
PathDisplay::PathDisplay()
{
  style_property_ = new rviz::EnumProperty("Line Style", "Lines",
      "The rendering operation to use to draw the grid lines.", this, SLOT(updateStyle()));
  style_property_->addOption("Lines", static_cast<int>(LineStyle::LINES));
  style_property_->addOption("Billboards", static_cast<int>(LineStyle::BILLBOARDS));

  line_width_property_ = new rviz::FloatProperty("Line Width", 0.03, "The width, in meters, of each path line. "
      "Only works with the 'Billboards' style.", this, SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.001);
  line_width_property_->hide();

  color_property_ = new rviz::ColorProperty("Color", QColor(41, 170, 226), "Color to draw the path.", this);
  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "Amount of transparency to apply to the path.", this);

  buffer_length_property_ = new rviz::IntProperty("Buffer Length", 1, "Number of paths to display.", this,
                                                  SLOT(updateBufferLength()));
  buffer_length_property_->setMin(1);

  offset_property_ = new rviz::VectorProperty("Offset", Ogre::Vector3::ZERO,
      "Allows you to offset the path from the origin of the reference frame.  In meters.", this, SLOT(updateOffset()));

  pose_style_property_ = new rviz::EnumProperty("Pose Style", "None", "Shape to display the pose as.", this,
                                                SLOT(updatePoseStyle()));
  pose_style_property_->addOption("None", static_cast<int>(PoseStyle::NONE));
  pose_style_property_->addOption("Axes", static_cast<int>(PoseStyle::AXES));
  pose_style_property_->addOption("Arrows", static_cast<int>(PoseStyle::ARROWS));

  pose_axes_length_property_ = new rviz::FloatProperty("Length", 0.3, "Length of the axes.", this,
                                                       SLOT(updatePoseAxisGeometry()));
  pose_axes_radius_property_ = new rviz::FloatProperty("Radius", 0.03, "Radius of the axes.", this,
                                                       SLOT(updatePoseAxisGeometry()));
  pose_arrow_color_property_ = new rviz::ColorProperty("Pose Color", QColor(41, 170, 226), "Color to draw the poses.",
                                                       this, SLOT(updatePoseArrowColor()));
  pose_arrow_shaft_length_property_ = new rviz::FloatProperty("Shaft Length", 0.1, "Length of the arrow shaft.",
                                                              this, SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_length_property_ = new rviz::FloatProperty("Head Length", 0.2, "Length of the arrow head.",
                                                              this, SLOT(updatePoseArrowGeometry()));
  pose_arrow_shaft_diameter_property_ = new rviz::FloatProperty("Shaft Diameter", 0.1, "Diameter of the arrow shaft.",
                                                                this, SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_diameter_property_ = new rviz::FloatProperty("Head Diameter", 0.3, "Diameter of the arrow head.",
                                                               this, SLOT(updatePoseArrowGeometry()));
  pose_axes_length_property_->hide();
  pose_axes_radius_property_->hide();
  pose_arrow_color_property_->hide();
  pose_arrow_shaft_length_property_->hide();
  pose_arrow_head_length_property_->hide();
  pose_arrow_shaft_diameter_property_->hide();
  pose_arrow_head_diameter_property_->hide();
}

PathDisplay::~PathDisplay()
{
  destroyObjects();
}

void PathDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
}

void PathDisplay::reset()
{
  MFDClass::reset();
  updateBufferLength();
}

void PathDisplay::allocateAxesVector(std::vector<rviz::Axes*>& axes_vect, int num)
{
  unsigned int num_u = static_cast<unsigned int>(num);
  if (num_u > axes_vect.size())
  {
    for (size_t i = axes_vect.size(); i < num_u; i++)
    {
      rviz::Axes* axes = new rviz::Axes(scene_manager_, scene_node_,
                                        pose_axes_length_property_->getFloat(),
                                        pose_axes_radius_property_->getFloat());
      axes_vect.push_back(axes);
    }
  }
  else if (num_u < axes_vect.size())
  {
    for (int i = axes_vect.size() - 1; num <= i; i--)
    {
      delete axes_vect[i];
    }
    axes_vect.resize(num);
  }
}

void PathDisplay::allocateArrowVector(std::vector<rviz::Arrow*>& arrow_vect, int num)
{
  unsigned int num_u = static_cast<unsigned int>(num);
  if (num_u > arrow_vect.size())
  {
    for (size_t i = arrow_vect.size(); i < num_u; i++)
    {
      rviz::Arrow* arrow = new rviz::Arrow(scene_manager_, scene_node_);
      arrow_vect.push_back(arrow);
    }
  }
  else if (num_u < arrow_vect.size())
  {
    for (int i = arrow_vect.size() - 1; num <= i; i--)
    {
      delete arrow_vect[i];
    }
    arrow_vect.resize(num);
  }
}

void PathDisplay::updateStyle()
{
  switch (getLineStyle())
  {
  case LineStyle::LINES:
  default:
    line_width_property_->hide();
    break;

  case LineStyle::BILLBOARDS:
    line_width_property_->show();
    break;
  }

  updateBufferLength();
}

void PathDisplay::updateLineWidth()
{
  float line_width = line_width_property_->getFloat();

  if (getLineStyle() == LineStyle::BILLBOARDS)
  {
    for (rviz::BillboardLine* billboard_line : billboard_lines_)
    {
      if (billboard_line) billboard_line->setLineWidth(line_width);
    }
  }
  context_->queueRender();
}

void PathDisplay::updateOffset()
{
  scene_node_->setPosition(offset_property_->getVector());
  context_->queueRender();
}

void PathDisplay::updatePoseStyle()
{
  switch (getPoseStyle())
  {
  case PoseStyle::AXES:
    pose_axes_length_property_->show();
    pose_axes_radius_property_->show();
    pose_arrow_color_property_->hide();
    pose_arrow_shaft_length_property_->hide();
    pose_arrow_head_length_property_->hide();
    pose_arrow_shaft_diameter_property_->hide();
    pose_arrow_head_diameter_property_->hide();
    break;
  case PoseStyle::ARROWS:
    pose_axes_length_property_->hide();
    pose_axes_radius_property_->hide();
    pose_arrow_color_property_->show();
    pose_arrow_shaft_length_property_->show();
    pose_arrow_head_length_property_->show();
    pose_arrow_shaft_diameter_property_->show();
    pose_arrow_head_diameter_property_->show();
    break;
  default:
    pose_axes_length_property_->hide();
    pose_axes_radius_property_->hide();
    pose_arrow_color_property_->hide();
    pose_arrow_shaft_length_property_->hide();
    pose_arrow_head_length_property_->hide();
    pose_arrow_shaft_diameter_property_->hide();
    pose_arrow_head_diameter_property_->hide();
  }
  updateBufferLength();
}

void PathDisplay::updatePoseAxisGeometry()
{
  for (auto& axes_vect : axes_chain_)
  {
    for (auto& axes : axes_vect)
    {
      axes->set(pose_axes_length_property_->getFloat(),
                pose_axes_radius_property_->getFloat());
    }
  }
  context_->queueRender();
}

void PathDisplay::updatePoseArrowColor()
{
  Ogre::ColourValue color = pose_arrow_color_property_->getOgreColor();

  for (auto& arrow_vect : arrow_chain_)
  {
    for (auto& arrow : arrow_vect)
    {
      arrow->setColor(color);
    }
  }
  context_->queueRender();
}

void PathDisplay::updatePoseArrowGeometry()
{
  for (auto& arrow_vect : arrow_chain_)
  {
    for (auto& arrow : arrow_vect)
    {
      arrow->set(pose_arrow_shaft_length_property_->getFloat(),
                 pose_arrow_shaft_diameter_property_->getFloat(),
                 pose_arrow_head_length_property_->getFloat(),
                 pose_arrow_head_diameter_property_->getFloat());
    }
  }
  context_->queueRender();
}

void PathDisplay::destroyObjects()
{
  // Destroy all simple lines, if any
  for (auto& manual_object : manual_objects_)
  {
    if (manual_object)
    {
      manual_object->clear();
      scene_manager_->destroyManualObject(manual_object);
      manual_object = NULL;  // ensure it doesn't get destroyed again
    }
  }

  // Destroy all billboards, if any
  for (auto& billboard_line : billboard_lines_)
  {
    if (billboard_line)
    {
      delete billboard_line;  // also destroys the corresponding scene node
      billboard_line = NULL;  // ensure it doesn't get destroyed again
    }
  }

  // Destroy Pose Vectors
  for (auto& axes_vect : axes_chain_)
  {
    allocateAxesVector(axes_vect, 0);
  }
  axes_chain_.resize(0);

  for (auto& arrow_vect : arrow_chain_)
  {
    allocateArrowVector(arrow_vect, 0);
  }
  arrow_chain_.resize(0);
}

void PathDisplay::updateBufferLength()
{
  // Delete old path objects
  destroyObjects();

  // Read options
  int buffer_length = buffer_length_property_->getInt();

  // Create new path objects
  switch (getLineStyle())
  {
  case LineStyle::LINES:  // simple lines with fixed width of 1px
    manual_objects_.resize(buffer_length);
    for (size_t i = 0; i < manual_objects_.size(); i++)
    {
      manual_objects_[i] = scene_manager_->createManualObject();
      manual_objects_[i]->setDynamic(true);
      scene_node_->attachObject(manual_objects_[i]);
    }
    break;

  case LineStyle::BILLBOARDS:  // billboards with configurable width
    billboard_lines_.resize(buffer_length);
    for (size_t i = 0; i < billboard_lines_.size(); i++)
    {
      billboard_lines_[i] = new rviz::BillboardLine(scene_manager_, scene_node_);
    }
    break;
  }
  axes_chain_.resize(buffer_length);
  arrow_chain_.resize(buffer_length);
}

void PathDisplay::processMessage(const nav_2d_msgs::Path2D::ConstPtr& msg)
{
  // Calculate index of oldest element in cyclic buffer
  size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();

  LineStyle style = getLineStyle();
  PoseStyle pose_style = getPoseStyle();
  Ogre::ManualObject* manual_object = NULL;
  rviz::BillboardLine* billboard_line = NULL;

  // Delete oldest element
  switch (style)
  {
  case LineStyle::LINES:
    manual_object = manual_objects_[bufferIndex];
    manual_object->clear();
    break;

  case LineStyle::BILLBOARDS:
    billboard_line = billboard_lines_[bufferIndex];
    billboard_line->clear();
    break;
  }

  std::vector<rviz::Arrow*>& arrow_vect = arrow_chain_[bufferIndex];
  std::vector<rviz::Axes*>& axes_vect = axes_chain_[bufferIndex];

  // Check if path contains invalid coordinate values
  if (!validateFloats(*msg))
  {
    setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Lookup transform into fixed frame
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
  }

  Ogre::Matrix4 transform(orientation);
  transform.setTrans(position);

  // scene_node_->setPosition(position);
  // scene_node_->setOrientation(orientation);

  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();
  Ogre::ColourValue arrow_color = pose_arrow_color_property_->getOgreColor();

  uint32_t num_points = msg->poses.size();
  switch (style)
  {
  case LineStyle::LINES:
    manual_object->estimateVertexCount(num_points);
    manual_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    break;

  case LineStyle::BILLBOARDS:
    billboard_line->setNumLines(1);
    billboard_line->setMaxPointsPerLine(num_points);
    billboard_line->setLineWidth(line_width_property_->getFloat());
    break;
  }
  switch (pose_style)
  {
  case PoseStyle::NONE:
    break;
  case PoseStyle::AXES:
    allocateAxesVector(axes_vect, num_points);
    break;
  case PoseStyle::ARROWS:
    allocateArrowVector(arrow_vect, num_points);
    break;
  }

  for (uint32_t i = 0; i < num_points; ++i)
  {
    geometry_msgs::Pose pose = nav_2d_utils::pose2DToPose(msg->poses[i]);

    Ogre::Vector3 xpos = transform * Ogre::Vector3(pose.position.x,
                                                   pose.position.y,
                                                   pose.position.z);
    const geometry_msgs::Quaternion& quat = pose.orientation;
    Ogre::Quaternion xquat = orientation * Ogre::Quaternion(quat.w, quat.x, quat.y, quat.z);

    switch (style)
    {
    case LineStyle::LINES:
      manual_object->position(xpos.x, xpos.y, xpos.z);
      manual_object->colour(color);
      break;
    case LineStyle::BILLBOARDS:
      billboard_line->addPoint(xpos, color);
      break;
    }
    switch (pose_style)
    {
    case PoseStyle::NONE:
      break;

    case PoseStyle::AXES:
      axes_vect[i]->setPosition(xpos);
      axes_vect[i]->setOrientation(xquat);
      break;

    case PoseStyle::ARROWS:
      arrow_vect[i]->setColor(arrow_color);
      arrow_vect[i]->set(pose_arrow_shaft_length_property_->getFloat(),
                         pose_arrow_shaft_diameter_property_->getFloat(),
                         pose_arrow_head_length_property_->getFloat(),
                         pose_arrow_head_diameter_property_->getFloat());
      arrow_vect[i]->setPosition(xpos);
      arrow_vect[i]->setDirection(xquat * Ogre::Vector3(1, 0, 0));
      break;
    }
  }

  if (style == LineStyle::LINES)
    manual_object->end();

  context_->queueRender();
}

}  // namespace robot_nav_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::PathDisplay, rviz::Display)
