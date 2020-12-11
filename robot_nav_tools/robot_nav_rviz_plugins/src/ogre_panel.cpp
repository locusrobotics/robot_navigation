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

#include <robot_nav_rviz_plugins/ogre_panel.h>
#include <nav_2d_utils/bounds.h>
#include <nav_2d_utils/conversions.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSceneNode.h>
#include <ros/ros.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace robot_nav_rviz_plugins
{

OgrePanel::OgrePanel(nav_grid::VectorNavGrid<unsigned char>& data,
                     Ogre::SceneManager& scene_manager, Ogre::SceneNode& scene_node)
  : data_(data), scene_manager_(scene_manager), scene_node_(scene_node)
{
}

void OgrePanel::updateInfo(const nav_grid::NavGridInfo& info)
{
  unsigned int rows = 1, cols = 1;
  nav_core2::UIntBounds original_bounds = nav_2d_utils::getFullUIntBounds(info);

  /*
   * Start by making one really large subpanel.
   * If that doesn't work (i.e. throws an exception) divide the panel by its largest dimension and
   * try again with more smaller subpanels.
   */
  while (true)
  {
    unsigned int n_swatches = rows * cols;
    ROS_DEBUG("Creating %d swatches", n_swatches);
    swatches_.clear();
    try
    {
      for (const auto& bounds : nav_2d_utils::divideBounds(original_bounds, cols, rows))
      {
        swatches_.push_back(std::make_shared<PartialOgrePanel>(scene_manager_, scene_node_, bounds, info.resolution));
      }
      // successfully created swatches without error. Return
      return;
    }
    catch (Ogre::RenderingAPIException&)
    {
      ROS_DEBUG("Failed to create %d swatches", n_swatches);
      // Divide by the largest dimension
      if (info.width / cols > info.height / rows)
        cols *= 2;
      else
        rows *= 2;
    }
  }
}


void OgrePanel::updateData(const nav_core2::UIntBounds& updated_bounds)
{
  // Copy the updated data to the partial panels
  std::vector<unsigned char> pixels;
  unsigned int updated_panels = 0;
  for (PartialOgrePanel::Ptr& swatch : swatches_)
  {
    const nav_core2::UIntBounds& swatch_bounds = swatch->getBounds();

    // If the bounds of the swatch do not overlap with updated area, we can skip updating this swatch
    if (!swatch_bounds.overlaps(updated_bounds)) continue;
    updated_panels++;

    unsigned int swatch_width = swatch_bounds.getWidth(),
                 swatch_height = swatch_bounds.getHeight();
    pixels.resize(swatch_width * swatch_height);

    auto pixel_it = pixels.begin();

    // The data for both data_ and the swatch_ are in row-major order, but with
    // possibly different row sizes, so we need to create the new pixels array for the
    // swatch with the smaller row size.
    unsigned int x_min = swatch_bounds.getMinX();
    unsigned int y_min = swatch_bounds.getMinY();
    unsigned int y_max = swatch_bounds.getMaxY();
    unsigned int width = data_.getWidth();
    for (unsigned int yy = y_min; yy <= y_max; yy++)
    {
      int index = yy * width + x_min;
      std::copy(&data_[index], &data_[index + swatch_width], pixel_it);
      pixel_it += swatch_width;
    }
    swatch->updateData(pixels);
  }
  ROS_DEBUG("Updated %d/%zu panels", updated_panels, swatches_.size());
}

void OgrePanel::addPalette(const NavGridPalette& palette)
{
  const std::string name = palette.getName();

  // Get Proper Number of Colors
  auto colors =  palette.getColors();
  if (colors.size() < NavGridPalette::NUM_COLORS)
  {
    ROS_DEBUG("Palette %s only has %zu colors defined. The rest will be black.",
              name.c_str(), colors.size());
    colors.resize(NavGridPalette::NUM_COLORS);
  }
  else if (colors.size() > NavGridPalette::NUM_COLORS)
  {
    ROS_WARN("Palette %s has %zu colors defined...can only use the first %d.",
             name.c_str(), colors.size(), NavGridPalette::NUM_COLORS);
    colors.resize(NavGridPalette::NUM_COLORS);
  }

  // Construct Ogre Palette
  Ogre::DataStreamPtr palette_stream;
  palette_stream.bind(new Ogre::MemoryDataStream(&colors[0],
      static_cast<size_t>(NavGridPalette::NUM_COLORS * NavGridPalette::NUM_CHANNELS)));
  static int palette_tex_count = 0;
  std::stringstream ss;
  ss << "NavGridPaletteTexture" << palette_tex_count++;

  palettes_[name] = Ogre::TextureManager::getSingleton().loadRawData(ss.str(),
         Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
         palette_stream, NavGridPalette::NUM_COLORS, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);

  palette_transparency_[name] = palette.hasTransparency();
}

void OgrePanel::setPalette(const std::string& palette_name)
{
  current_palette_ = palette_name;
  std::string ogre_palette_name = palettes_[palette_name]->getName();
  for (PartialOgrePanel::Ptr& swatch : swatches_)
  {
    swatch->setTexture(ogre_palette_name, 1);
  }
}

void OgrePanel::updateAlpha(float alpha, bool draw_behind)
{
  Ogre::SceneBlendType scene_blending;
  bool depth_write;
  int group = draw_behind ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;

  if (alpha < 1.0 || palette_transparency_[current_palette_])
  {
    scene_blending = Ogre::SBT_TRANSPARENT_ALPHA;
    depth_write = false;
  }
  else
  {
    scene_blending = Ogre::SBT_REPLACE;
    depth_write = !draw_behind;
  }

  // helper class to set alpha parameter on all renderables.
  class AlphaSetter: public Ogre::Renderable::Visitor
  {
  public:
    explicit AlphaSetter(float alpha)
      : alpha_vec_(alpha, alpha, alpha, alpha)
    {}

    void visit(Ogre::Renderable *rend, ushort lodIndex, bool isDebug, Ogre::Any *pAny = 0)
    {
      rend->setCustomParameter(ALPHA_PARAMETER, alpha_vec_);
    }
  private:
    Ogre::Vector4 alpha_vec_;
  };

  AlphaSetter alpha_setter(alpha);
  for (PartialOgrePanel::Ptr& swatch : swatches_)
  {
    swatch->updateAlphaRendering(scene_blending, depth_write, group, &alpha_setter);
  }
}

void OgrePanel::clear()
{
  for (PartialOgrePanel::Ptr& swatch : swatches_)
  {
    swatch->clear();
  }
}

bool OgrePanel::transformMap(rviz::FrameManager& fm)
{
  // Translate origin of nav grid to ogre coordinates
  geometry_msgs::Pose origin = nav_2d_utils::getOrigin3D(data_.getInfo());

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  bool ret = fm.transform(data_.getFrameId(), ros::Time(0), origin, position, orientation);

  scene_node_.setPosition(position);
  scene_node_.setOrientation(orientation);

  return ret;
}
}  // namespace robot_nav_rviz_plugins
