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

#ifndef ROBOT_NAV_RVIZ_PLUGINS_NAV_GRID_DISPLAY_H
#define ROBOT_NAV_RVIZ_PLUGINS_NAV_GRID_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#endif

#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/display.h>

#include <nav_grid/vector_nav_grid.h>

#include <robot_nav_rviz_plugins/ogre_panel.h>
#include <robot_nav_rviz_plugins/nav_grid_palette.h>
#include <pluginlib/class_loader.h>

#include <string>
#include <vector>

namespace robot_nav_rviz_plugins
{
/**
 * @brief Displays a nav_grid (of unspecified type) along the XY plane.
 *
 * The actual object is rendered using the OgrePanel class. This class contains the rviz/QT logic for
 * displaying the options/properties in the Displays list and connecting changes there to the appropriate events.
 *
 * This class does not subscribe to any data. That functionality is relegated to subclasses which will specify the
 * underlying data type and possibly translate it.
 */
class NavGridDisplay: public rviz::Display
{
Q_OBJECT
public:
  /**
   * @brief Constructor for the display
   * @param data_type The string representing the datatype (needed for setting up topic_property_)
   * @param include_ignore_property Whether to include the ignore property, which is only needed by some subclasses
   *
   * Specifying the ignore_property_ in the subclass would necessitate defining a separate MOC generating header
   * for each subclass, which is a little bit of a pain, so instead, it is included in this, the parent class for
   * a bit of cleanliness.
   */
  explicit NavGridDisplay(const std::string& data_type, bool include_ignore_property = false);
  virtual ~NavGridDisplay();

  /**
   * @name Public overrides from Display
   * @{
   */
  void reset() override;
  void setTopic(const QString &topic, const QString &datatype) override;
  void update(float wall_dt, float ros_dt) override;
  /**
   * @}
   */

Q_SIGNALS:
  /**
   * @brief Custom signal emitted when new map data is received
   */
  void mapUpdated(const nav_core2::UIntBounds& updated_bounds);

protected Q_SLOTS:
  /**
   * @name Custom events triggered by changing properties
   * @{
   */
  void updateAlpha();
  void updateTopic();
  void updatePalette();
  void showMap(const nav_core2::UIntBounds& updated_bounds);
  virtual void updateIgnoreType();
  virtual void updateIgnore() {}
  /**
   * @}
   */

protected:
  enum struct IgnoreType {NONE, VALUE, LIMIT};
  IgnoreType getIgnoreType() const { return static_cast<IgnoreType>(ignore_type_property_->getOptionInt()); }

  /**
   * @name Protected overrides from Display
   * @{
   */
  void onInitialize() override;
  void fixedFrameChanged() override;
  void onEnable() override;
  void onDisable() override;
  /**
   * @}
   */

  /**
   * @brief Called to trigger subscription, handles empty topics setting status
   *
   * Actual subscription logic should be handled by subclass via onSubscribe method
   */
  void subscribe();

  /**
   * @brief Actual subscription logic, called by subscribe
   * @param topic Nonempty string with topic to subscribe to
   */
  virtual void onSubscribe(const std::string& topic) {}

  /**
   * @brief Called to trigger unsubscribing.
   *
   * Actual unsubscribing logic should be handled by subclass via onUnsubscribe method
   */
  void unsubscribe();

  /**
   * @brief Actual unsubscription logic, called by unsubscribe.
   */
  virtual void onUnsubscribe() {}

  /**
   * @brief Clear the data and remove the objects from the screen
   */
  void clear();

  /**
   * @brief Put the map in its proper place
   */
  void transformMap();

  // The Actual Display Object
  // Note it has to be a pointer for lazy initialization
  OgrePanel::Ptr panel_display_;

  // Data
  nav_grid::VectorNavGrid<unsigned char> panel_data_;
  nav_grid::NavGridInfo cached_info_;

  // Non-editiable properties
  rviz::FloatProperty* resolution_property_;
  rviz::IntProperty* width_property_;
  rviz::IntProperty* height_property_;

  // Editable Properties
  rviz::RosTopicProperty* topic_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::BoolProperty* unreliable_property_;
  rviz::BoolProperty* draw_behind_property_;
  rviz::EnumProperty* color_scheme_property_;

  // Properties only used by some subclasses (see constructor)
  rviz::EnumProperty* ignore_type_property_;
  rviz::FloatProperty* ignore_property_;

  // Color Scheme Management
  std::vector<std::string> color_scheme_names_;
  pluginlib::ClassLoader<NavGridPalette> palette_loader_;
};

}  // namespace robot_nav_rviz_plugins

#endif  // ROBOT_NAV_RVIZ_PLUGINS_NAV_GRID_DISPLAY_H
