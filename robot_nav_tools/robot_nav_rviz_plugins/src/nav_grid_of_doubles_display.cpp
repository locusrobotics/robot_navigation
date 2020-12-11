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
#include <nav_grid_iterators/sub_grid.h>
#include <nav_2d_msgs/NavGridOfDoubles.h>
#include <nav_2d_msgs/NavGridOfDoublesUpdate.h>
#include <nav_2d_utils/bounds.h>
#include <nav_grid_pub_sub/nav_grid_subscriber.h>
#include <limits>
#include <string>

namespace robot_nav_rviz_plugins
{
/**
 * @brief Displays a nav_grid of doubles along the XY plane.
 *
 * Unlike NavGridOfCharsDisplay, this requires a bit more processing before writing the data into `panel_data_.
 * The double data must be scaled to values [0, 255] so as to map to colors in the palette. This requires tracking the
 * extreme values of the grid to the best of our ability and scaling accordingly.
 *
 * If only NavGridOfDoubles messages (with no NavGridOfDoublesUpdate messages) are published, then the min/max vals will
 * be accurate. However, if update messages are published too, its possible that the update could overwrite the previous
 * extreme values.
 *
 * Consider a simple 4x1 grid containing [0.0, 10.0, 5.0, 5.0]. The min/max values will be 0.0 and 10.0 respectively.
 * However, if an update is recieved that updates the first two values only to 6.0, the new grid will contain
 * [6.0, 6.0, 5.0, 5.0] but the min/max will still be 0.0/10.0, because the entire grid will not be rescanned.
 */
class NavGridOfDoublesDisplay: public NavGridDisplay
{
public:
  NavGridOfDoublesDisplay()
    : NavGridDisplay(ros::message_traits::datatype<nav_2d_msgs::NavGridOfDoubles>(), true)
    , sub_(double_data_)  // set up the subscriber to update into `double_data_` which is translated to `panel_data_`
    , first_data_(true)
  {
    // These properties can be defined here (and not the parent class) since they don't require qt events
    min_property_ = new rviz::FloatProperty("Min Value", 0.0, "Minimum value in the grid (not editable)", this);
    min_property_->setReadOnly(true);
    max_property_ = new rviz::FloatProperty("Max Value", 0.0, "Maximum value in the grid (not editable)", this);
    max_property_->setReadOnly(true);

    resetExtremeValues();
    updateIgnore();
  }

  void onSubscribe(const std::string& topic) override
  {
    resetExtremeValues();
    sub_.init(update_nh_, std::bind(&NavGridOfDoublesDisplay::newDataCallback, this, std::placeholders::_1),
              topic, true, true);
  }

  void onUnsubscribe() override
  {
    sub_.deactivate();
  }

  void updateIgnore()
  {
    ignore_value_ = ignore_property_->getFloat();
    resetExtremeValues();
  }
protected:
  void resetExtremeValues()
  {
    min_value_ = std::numeric_limits<double>::max();
    max_value_ = -min_value_;
  }

  void newDataCallback(const nav_core2::UIntBounds& bounds)
  {
    if (bounds.isEmpty())
    {
      return;
    }

    nav_grid::NavGridInfo info = double_data_.getInfo();

    if (first_data_)
    {
      panel_data_.setInfo(info);
      first_data_ = false;
    }
    else
    {
      panel_data_.updateInfo(info);
    }
    nav_core2::UIntBounds full_bounds = nav_2d_utils::getFullUIntBounds(info);

    bool extremes_changed = false;
    if (bounds == full_bounds)
    {
      // If the update covers the whole grid, start extreme value calculation from scratch
      resetExtremeValues();
    }

    // Search the updated area for new min/max values while ignoring `ignore_value_`.
    IgnoreType ignore_type = getIgnoreType();
    for (const nav_grid::Index& i : nav_grid_iterators::SubGrid(&info, bounds))
    {
      double value = double_data_(i);
      if ((ignore_type == IgnoreType::VALUE && value == ignore_value_) ||
          (ignore_type == IgnoreType::LIMIT && value >= ignore_value_))
      {
        continue;
      }
      else if (value < min_value_)
      {
        min_value_ = value;
        extremes_changed = true;
      }
      else if (value > max_value_)
      {
        max_value_ = value;
        extremes_changed = true;
      }
    }

    nav_core2::UIntBounds updated_bounds;
    // If the extremes changed, then all the values need to be rescaled
    if (extremes_changed)
    {
      updated_bounds.merge(full_bounds);
      min_property_->setValue(min_value_);
      max_property_->setValue(max_value_);
    }
    else
    {
      updated_bounds.merge(bounds);
    }

    double denominator = max_value_ - min_value_;
    // Ensure safe division
    if (denominator == 0)
    {
      denominator = 1;
    }

    for (const nav_grid::Index& i : nav_grid_iterators::SubGrid(&info, updated_bounds))
    {
      if ((ignore_type == IgnoreType::VALUE && double_data_(i) == ignore_value_) ||
          (ignore_type == IgnoreType::LIMIT && double_data_(i) >= ignore_value_))
      {
        panel_data_.setValue(i, 0);
      }
      else
      {
        double ratio = (double_data_(i) - min_value_) / denominator;
        panel_data_.setValue(i, 1 + static_cast<unsigned char>(254 * ratio));
      }
    }

    Q_EMIT mapUpdated(updated_bounds);
  }

  nav_grid::VectorNavGrid<double> double_data_;
  nav_grid_pub_sub::NavGridOfDoublesSubscriber sub_;
  double min_value_, max_value_, ignore_value_;
  rviz::FloatProperty* min_property_;
  rviz::FloatProperty* max_property_;

  bool first_data_;
};
}  // namespace robot_nav_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::NavGridOfDoublesDisplay, rviz::Display)
