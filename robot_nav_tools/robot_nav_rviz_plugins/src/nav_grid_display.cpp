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

#include <robot_nav_rviz_plugins/nav_grid_display.h>
#include <robot_nav_rviz_plugins/validate_floats.h>
#include <boost/bind.hpp>

#include <OgreSharedPtr.h>
#include <OgreVector3.h>

#include <ros/ros.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rviz/ogre_helpers/grid.h>
#include <rviz/display_context.h>


namespace robot_nav_rviz_plugins
{
NavGridDisplay::NavGridDisplay(const std::string& data_type, bool include_ignore_property)
  : Display()
  , panel_display_(nullptr)
  , palette_loader_("robot_nav_rviz_plugins", "robot_nav_rviz_plugins::NavGridPalette")
{
  connect(this, SIGNAL(mapUpdated(nav_core2::UIntBounds)), this, SLOT(showMap(nav_core2::UIntBounds)));

  topic_property_ = new rviz::RosTopicProperty("Topic", "",
      QString::fromStdString(data_type), QString::fromStdString(data_type + std::string(" topic to subscribe to.")),
      this, SLOT(updateTopic()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 0.7,
      "Amount of transparency to apply to the map.",
      this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  color_scheme_property_ = new rviz::EnumProperty("Color Scheme", "map", "How to color the occupancy values.",
      this, SLOT(updatePalette()));

  draw_behind_property_ = new rviz::BoolProperty("Draw Behind", false,
      "Rendering option, controls whether or not the map is always"
      " drawn behind everything else.",
      this, SLOT(updateAlpha()));

  resolution_property_ = new rviz::FloatProperty("Resolution", 0, "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  width_property_ = new rviz::IntProperty("Width", 0, "Width of the map, in cells. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new rviz::IntProperty("Height", 0, "Height of the map, in cells. (not editable)", this);
  height_property_->setReadOnly(true);

  unreliable_property_ = new rviz::BoolProperty("Unreliable", false,
      "Prefer UDP topic transport",
      this,
      SLOT(updateTopic()));

  if (include_ignore_property)
  {
    ignore_type_property_ = new rviz::EnumProperty("Ignore Value Type", "None", "Way to exclude certain value(s)", this,
                                                  SLOT(updateIgnoreType()));
    ignore_type_property_->addOption("None", static_cast<int>(IgnoreType::NONE));
    ignore_type_property_->addOption("Value", static_cast<int>(IgnoreType::VALUE));
    ignore_type_property_->addOption("Limit", static_cast<int>(IgnoreType::LIMIT));

    ignore_property_ = new rviz::FloatProperty("Ignore Value", -1.0, "Value to not include in the min/max",
        this, SLOT(updateIgnore()));
    ignore_property_->hide();
  }
}

NavGridDisplay::~NavGridDisplay()
{
  unsubscribe();
  clear();
}

/**********************************************************************************
 * Overrides from Display
 **********************************************************************************/
void NavGridDisplay::onInitialize()
{
  // Lazy initialization of the panel_display_ to ensure we have the non-null pointers to the ogre objects
  if (panel_display_) return;

  panel_display_ = std::make_shared<OgrePanel>(panel_data_, *scene_manager_, *scene_node_);

  for (auto plugin_name : palette_loader_.getDeclaredClasses())
  {
    auto palette = palette_loader_.createInstance(plugin_name);

    std::string name = palette->getName();
    color_scheme_property_->addOption(name.c_str(), static_cast<int>(color_scheme_names_.size()));
    color_scheme_names_.push_back(name);
    panel_display_->addPalette(*palette);
  }
  // Set the current values
  updatePalette();
}

void NavGridDisplay::onEnable()
{
  subscribe();
}

void NavGridDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void NavGridDisplay::reset()
{
  Display::reset();

  // Force resubscription so that the map will be re-sent
  updateTopic();
}

void NavGridDisplay::setTopic(const QString &topic, const QString &datatype)
{
  topic_property_->setString(topic);
}

void NavGridDisplay::update(float wall_dt, float ros_dt)
{
  // periodically make sure the map is in the right spot
  transformMap();
}

void NavGridDisplay::fixedFrameChanged()
{
  // retransform if frame changed
  transformMap();
}

/**********************************************************************************
 * Custom Events triggered by properties
 **********************************************************************************/
void NavGridDisplay::updateAlpha()
{
  panel_display_->updateAlpha(alpha_property_->getFloat(), draw_behind_property_->getValue().toBool());
}

void NavGridDisplay::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
}

void NavGridDisplay::updatePalette()
{
  int palette_index = color_scheme_property_->getOptionInt();
  panel_display_->setPalette(color_scheme_names_[palette_index]);
  updateAlpha();
}

void NavGridDisplay::showMap(const nav_core2::UIntBounds& updated_bounds)
{
  if (updated_bounds.isEmpty())
  {
    return;
  }

  nav_grid::NavGridInfo info = panel_data_.getInfo();
  // Only process the info if the info has changed
  if (info != cached_info_)
  {
    if (!validateFloats(info))
    {
      setStatus(rviz::StatusProperty::Error, "Map", "Message contained invalid floating point values (nans or infs)");
      return;
    }

    if (info.width * info.height == 0)
    {
      std::stringstream ss;
      ss << "Map is zero-sized (" << info.width << "x" << info.height << ")";
      setStatus(rviz::StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
      return;
    }

    // If the data has changed size or resolution, update the panel display's info
    if (info.resolution != cached_info_.resolution ||
        info.width != cached_info_.width || info.height != cached_info_.height)
    {
      panel_display_->updateInfo(info);
      resolution_property_->setValue(info.resolution);
      width_property_->setValue(info.width);
      height_property_->setValue(info.height);
    }
    cached_info_ = info;
  }

  setStatus(rviz::StatusProperty::Ok, "Message", "Map received");

  panel_display_->updateData(updated_bounds);

  updatePalette();

  transformMap();

  setStatus(rviz::StatusProperty::Ok, "Map", "Map OK");
  context_->queueRender();
}


void NavGridDisplay::updateIgnoreType()
{
  switch (getIgnoreType())
  {
  case IgnoreType::VALUE:
  case IgnoreType::LIMIT:
    ignore_property_->show();
    break;
  default:
    ignore_property_->hide();
  }
  updateIgnore();
}


/**********************************************************************************
 * Other Methods
 **********************************************************************************/

void NavGridDisplay::subscribe()
{
  // Make sure the topic is enabled and non-empty.
  // Also sets status if there is a problem

  if (!isEnabled())
  {
    return;
  }

  if (!topic_property_->getTopic().isEmpty())
  {
    try
    {
      onSubscribe(topic_property_->getTopicStd());
      setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void NavGridDisplay::unsubscribe()
{
  onUnsubscribe();
}

void NavGridDisplay::clear()
{
  setStatus(rviz::StatusProperty::Warn, "Message", "No map received");
  panel_display_->clear();
}

void NavGridDisplay::transformMap()
{
  if (panel_display_->transformMap(*context_->getFrameManager()))
  {
    setStatus(rviz::StatusProperty::Ok, "Transform", "Transform OK");
  }
  else
  {
    std::string frame = panel_data_.getFrameId();
    setStatus(rviz::StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame) + "] to [" + fixed_frame_ + "]");
  }
}
}  // namespace robot_nav_rviz_plugins
