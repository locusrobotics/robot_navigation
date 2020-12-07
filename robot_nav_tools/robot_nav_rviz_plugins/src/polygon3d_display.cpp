/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <robot_nav_rviz_plugins/polygon3d_display.h>
#include <robot_nav_rviz_plugins/validate_floats.h>
#include <nav_2d_utils/conversions.h>

namespace robot_nav_rviz_plugins
{
Polygon3DDisplay::Polygon3DDisplay()
{
  mode_property_ = new PolygonDisplayModeProperty(this, SLOT(updateStyle()));
  outline_color_property_ = new rviz::ColorProperty("Outline Color", QColor(36, 64, 142), "Color to draw the polygon.",
                                            this, SLOT(queueRender()));

  filler_color_property_ = new rviz::ColorProperty("Fill Color", QColor(165, 188, 255), "Color to fill the polygon.",
                                            this, SLOT(queueRender()));
  filler_alpha_property_ = new rviz::FloatProperty("Alpha", 0.8, "Amount of transparency to apply to the filler.",
                                            this, SLOT(queueRender()));
  filler_alpha_property_->setMin(0.0);
  filler_alpha_property_->setMax(1.0);

  zoffset_property_ = new rviz::FloatProperty("Z-Offset", 0.0, "Offset in the Z direction.", this, SLOT(queueRender()));
}

Polygon3DDisplay::~Polygon3DDisplay()
{
  if (outline_object_)
  {
    delete outline_object_;
  }
  if (filler_object_)
  {
    delete filler_object_;
  }
}

void Polygon3DDisplay::onInitialize()
{
  MFDClass::onInitialize();
  outline_object_ = new PolygonOutline(*scene_manager_, *scene_node_);
  filler_object_ = new PolygonFill(*scene_manager_, *scene_node_, polygon_material_.getName());
}

void Polygon3DDisplay::reset()
{
  MFDClass::reset();
  if (outline_object_) outline_object_->reset();
  if (filler_object_) filler_object_->reset();
}


void Polygon3DDisplay::updateStyle()
{
  if (mode_property_->shouldDrawOutlines())
  {
    outline_color_property_->show();
  }
  else
  {
    outline_color_property_->hide();
  }

  if (mode_property_->shouldDrawFiller())
  {
    filler_color_property_->show();
    filler_alpha_property_->show();
  }
  else
  {
    filler_color_property_->hide();
    filler_alpha_property_->hide();
  }
  queueRender();
}


void Polygon3DDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
  nav_2d_msgs::Polygon2DStamped polygon2d = nav_2d_utils::polygon3Dto2D(*msg);

  if (!validateFloats(polygon2d.polygon))
  {
    setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(polygon2d.header, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              polygon2d.header.frame_id.c_str(), qPrintable(fixed_frame_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
  bool empty = polygon2d.polygon.points.empty();

  double z_offset = zoffset_property_->getFloat();

  outline_object_->reset();
  if (mode_property_->shouldDrawOutlines() && !empty)
  {
    Ogre::ColourValue outline_color = rviz::qtToOgre(outline_color_property_->getColor());
    outline_object_->setPolygon(polygon2d.polygon, outline_color, z_offset);
  }

  if (!mode_property_->shouldDrawFiller() || empty)
  {
    filler_object_->reset();
  }
  else
  {
    filler_object_->setPolygon(polygon2d.polygon, getColor(filler_color_property_, filler_alpha_property_), z_offset);
  }
}

}  // namespace robot_nav_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::Polygon3DDisplay, rviz::Display)
