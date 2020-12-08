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

#include <robot_nav_rviz_plugins/polygons_display.h>
#include <robot_nav_rviz_plugins/validate_floats.h>
#include <color_util/convert.h>
#include <color_util/named_colors.h>
#include <vector>



namespace robot_nav_rviz_plugins
{
PolygonsDisplay::PolygonsDisplay()
{
  mode_property_ = new PolygonDisplayModeProperty(this, SLOT(updateStyle()));
  outline_color_property_ = new rviz::ColorProperty("Outline Color", QColor(79, 98, 142), "Color to draw the polygon.",
                                            this, SLOT(updateProperties()));

  color_mode_property_ = new rviz::EnumProperty("Fill Color Mode", "Single Color",
       "Color scheme for coloring each polygon", this, SLOT(updateStyle()));
  color_mode_property_->addOption("Single Color", static_cast<int>(FillColorMode::SINGLE));
  color_mode_property_->addOption("From Message", static_cast<int>(FillColorMode::FROM_MSG));
  color_mode_property_->addOption("Unique", static_cast<int>(FillColorMode::UNIQUE));

  filler_color_property_ = new rviz::ColorProperty("Fill Color", QColor(22, 41, 85), "Color to fill the polygon.",
                                            this, SLOT(updateProperties()));
  filler_alpha_property_ = new rviz::FloatProperty("Alpha", 0.8, "Amount of transparency to apply to the filler.",
                                            this, SLOT(updateProperties()));
  filler_alpha_property_->setMin(0.0);
  filler_alpha_property_->setMax(1.0);

  zoffset_property_ = new rviz::FloatProperty("Z-Offset", 0.0, "Offset in the Z direction.", this,
                                              SLOT(updateProperties()));

  for (const auto& color : color_util::getNamedColors())
  {
    if (color.a == 0.0) continue;
    unique_colors_.push_back(color_util::toMsg(color));
  }
}

PolygonsDisplay::~PolygonsDisplay()
{
  for (auto& outline_object : outline_objects_)
  {
    delete outline_object;
  }
  for (auto filler_object : filler_objects_)
  {
    delete filler_object;
  }
}

void PolygonsDisplay::reset()
{
  MFDClass::reset();
  resetOutlines();
  resetFillers();
}

void PolygonsDisplay::resetOutlines()
{
  for (auto& outline_object : outline_objects_)
  {
    outline_object->reset();
  }
}

void PolygonsDisplay::resetFillers()
{
  for (auto filler_object : filler_objects_)
  {
    filler_object->reset();
  }
}

void PolygonsDisplay::updateStyle()
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
    color_mode_property_->show();

    FillColorMode coloring = getFillColorMode();
    if (coloring == FillColorMode::SINGLE)
    {
      filler_color_property_->show();
      filler_alpha_property_->show();
    }
    else
    {
      filler_color_property_->hide();
      filler_alpha_property_->hide();
    }
  }
  else
  {
    color_mode_property_->hide();
    filler_color_property_->hide();
    filler_alpha_property_->hide();
  }
  updateProperties();
}

void PolygonsDisplay::processMessage(const nav_2d_msgs::Polygon2DCollection::ConstPtr& msg)
{
  for (const auto& polygon : msg->polygons)
  {
    if (!validateFloats(polygon))
    {
      setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
      return;
    }
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  // Save data internally
  saved_polygons_ = *msg;
  saved_outlines_.clear();
  for (const auto& complex : msg->polygons)
  {
    saved_outlines_.push_back(complex.outer);
    for (const auto& inner : complex.inner)
    {
      saved_outlines_.push_back(inner);
    }
  }

  // Resize object vectors
  const unsigned int num_outlines = saved_outlines_.size();
  while (outline_objects_.size() > num_outlines)
  {
    delete outline_objects_.back();
    outline_objects_.pop_back();
  }
  while (outline_objects_.size() < num_outlines)
  {
    outline_objects_.push_back(new PolygonOutline(*scene_manager_, *scene_node_));
  }

  const unsigned int num_fillers = msg->polygons.size();
  while (filler_objects_.size() > num_fillers)
  {
    delete filler_objects_.back();
    filler_objects_.pop_back();
  }
  while (filler_objects_.size() < num_fillers)
  {
    filler_objects_.push_back(new PolygonFill(*scene_manager_, *scene_node_, polygon_material_.getName()));
  }

  updateProperties();
}

void PolygonsDisplay::updateProperties()
{
  double z_offset = zoffset_property_->getFloat();
  bool empty = saved_polygons_.polygons.empty();

  resetOutlines();
  if (mode_property_->shouldDrawOutlines() && !empty)
  {
    Ogre::ColourValue outline_color = rviz::qtToOgre(outline_color_property_->getColor());
    for (unsigned int i = 0; i < saved_outlines_.size(); ++i)
    {
      outline_objects_[i]->setPolygon(saved_outlines_[i], outline_color, z_offset);
    }
  }

  if (!mode_property_->shouldDrawFiller() || empty)
  {
    resetFillers();
  }
  else
  {
    FillColorMode coloring = getFillColorMode();
    std_msgs::ColorRGBA default_color;  // transparent by default
    if (coloring == FillColorMode::SINGLE)
    {
      default_color = getColor(filler_color_property_, filler_alpha_property_);
    }

    for (unsigned int i = 0; i < saved_polygons_.polygons.size(); ++i)
    {
      std_msgs::ColorRGBA color;
      if (coloring == FillColorMode::UNIQUE)
      {
        color = unique_colors_[i % unique_colors_.size()];
      }
      else if (coloring == FillColorMode::FROM_MSG && i < saved_polygons_.colors.size())
      {
        color = saved_polygons_.colors[i];
      }
      else
      {
        color = default_color;
      }
      filler_objects_[i]->setPolygon(saved_polygons_.polygons[i], color, z_offset);
    }
  }
  queueRender();
}

}  // namespace robot_nav_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::PolygonsDisplay, rviz::Display)
