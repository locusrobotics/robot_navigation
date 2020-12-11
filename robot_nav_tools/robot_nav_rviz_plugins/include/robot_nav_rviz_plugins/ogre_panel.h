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

#ifndef ROBOT_NAV_RVIZ_PLUGINS_OGRE_PANEL_H
#define ROBOT_NAV_RVIZ_PLUGINS_OGRE_PANEL_H

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <nav_grid/vector_nav_grid.h>
#include <nav_core2/bounds.h>
#include <rviz/ogre_helpers/custom_parameter_indices.h>  // ALPHA_PARAMETER
#include <rviz/frame_manager.h>
#include <robot_nav_rviz_plugins/nav_grid_palette.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace robot_nav_rviz_plugins
{
/**
 * @brief A collection of Ogre objects for rendering a two-dimensional rectangular plane (such as a map) in rviz
 *
 * If the panel is sufficiently small, it can be done in a single Ogre::ManualObject. However, if it gets too big
 * then Ogre has difficulty rendering such a large texture. Hence, the OgrePanel is divided up into smaller
 * PartialOgrePanels, each of which contains a single ManualObject.
 * See https://github.com/ros-visualization/rviz/pull/1007 for more details.
 *
 * The pixels in the panel are determined by two things, the NavGrid data and the palette. The data provides a mapping
 * from each cell to a unsigned char (0-255), and the palette provides a mapping from the unsigned char to a RGBA color.
 */
class OgrePanel
{
public:
  OgrePanel(nav_grid::VectorNavGrid<unsigned char>& data,
            Ogre::SceneManager& scene_manager, Ogre::SceneNode& scene_node);

  /**
   * @brief Update the location/shape of the grid
   * @param info New shape
   */
  void updateInfo(const nav_grid::NavGridInfo& info);

  /**
   * @brief Update the panel data within the given bounds.
   *
   * Assumes underlying data object's values have already changed and this class just updates the Ogre objects
   * @param updated_bounds Bounds of the grid that have been updated
   */
  void updateData(const nav_core2::UIntBounds& updated_bounds);

  /**
   * @brief Add a palette with the given name
   */
  void addPalette(const NavGridPalette& palette);

  /**
   * @brief Set the palette to use
   * @param palette_name Name of the pallete (needs to be added previously with addPalette)
   */
  void setPalette(const std::string& palette_name);

  /**
   * @brief Sets the alpha/draw_behind properties
   * @param alpha The desired alpha channel value for the panel. [0.0, 1.0]
   * @param draw_behind Whether it should be drawn before (behind) the other maps
   */
  void updateAlpha(float alpha, bool draw_behind);

  /**
   * @brief Clear the panel from the visualization by making it invisible
   */
  void clear();

  /**
   * @brief Move the map to its proper position and orientation
   * @param fm Frame manager with TF info
   * @return Whether the correct frame was found
   */
  bool transformMap(rviz::FrameManager& fm);

  using Ptr = std::shared_ptr<OgrePanel>;

protected:
  /**
   * @brief Portion of the larger OgrePanel....see above
   */
  class PartialOgrePanel
  {
  public:
    PartialOgrePanel(Ogre::SceneManager& scene_manager, Ogre::SceneNode& parent_scene_node,
                        const nav_core2::UIntBounds& bounds, float resolution);
    ~PartialOgrePanel();

    const nav_core2::UIntBounds& getBounds() const { return bounds_; }

    /**
     * @brief Update the data for this Partial Panel.
     * @param pixels only contains the pixels (in row major order) for this particular partial panel
     */
    void updateData(std::vector<unsigned char>& pixels);
    void clear();

    void setTexture(const std::string& texture_name, int index);
    void updateAlphaRendering(Ogre::SceneBlendType scene_blending, bool depth_write, int group,
                              Ogre::Renderable::Visitor* alpha_setter);


    using Ptr = std::shared_ptr<PartialOgrePanel>;

  protected:
    Ogre::SceneManager& scene_manager_;
    Ogre::ManualObject* manual_object_;
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_;
    Ogre::SceneNode* scene_node_;
    nav_core2::UIntBounds bounds_;
  };

  nav_grid::VectorNavGrid<unsigned char>& data_;  // VectorNavGrid is used for easy copying
  Ogre::SceneManager& scene_manager_;
  Ogre::SceneNode& scene_node_;
  std::vector<PartialOgrePanel::Ptr> swatches_;  // For some reason, code segfaults if this is changed to raw object

  std::map<std::string, Ogre::TexturePtr> palettes_;
  std::map<std::string, bool> palette_transparency_;
  std::string current_palette_;
};
}  // namespace robot_nav_rviz_plugins

#endif  // ROBOT_NAV_RVIZ_PLUGINS_OGRE_PANEL_H
