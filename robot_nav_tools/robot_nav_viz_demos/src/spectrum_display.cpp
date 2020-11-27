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

#include <robot_nav_viz_demos/spectrum_display.h>
#include <boost/bind.hpp>
#include <color_util/blend.h>
#include <color_util/convert.h>
#include <algorithm>
#include <vector>
#include <OgreMatrix4.h>

namespace robot_nav_rviz_plugins
{
SpectrumDisplay::SpectrumDisplay()
{
  spectrum_property_ = new rviz::EnumProperty("Spectrum Style", "RGB",
      "The rendering operation to use to draw the grid lines.", this, SLOT(updateColors()));
  spectrum_property_->addOption("RGB", static_cast<int>(SpectrumStyle::rgb));
  spectrum_property_->addOption("HSV", static_cast<int>(SpectrumStyle::hsv));
  spectrum_property_->addOption("HSV+", static_cast<int>(SpectrumStyle::short_hsv));

  color_a_property_ = new rviz::ColorProperty("Color A", QColor(239, 41, 41), "Color A", this, SLOT(updateColors()));
  alpha_a_property_ = new rviz::FloatProperty("Alpha A", 1.0, "Alpha A", this, SLOT(updateColors()));
  alpha_a_property_->setMin(0.0);
  alpha_a_property_->setMax(1.0);
  color_b_property_ = new rviz::ColorProperty("Color B", QColor(41, 0, 226), "Color B", this, SLOT(updateColors()));
  alpha_b_property_ = new rviz::FloatProperty("Alpha B", 1.0, "Alpha B", this, SLOT(updateColors()));
  alpha_b_property_->setMin(0.0);
  alpha_b_property_->setMax(1.0);

  size_property_ = new rviz::IntProperty("Spectrum Size", 10, "Number of colors to display.", this, SLOT(updateSize()));
  size_property_->setMin(2);
}

void SpectrumDisplay::onInitialize()
{
  Display::onInitialize();
  updateSize();
}

void SpectrumDisplay::reset()
{
  Display::reset();
  updateSize();
}

void SpectrumDisplay::updateColors()
{
  Ogre::ColourValue ogre_a = color_a_property_->getOgreColor();
  Ogre::ColourValue ogre_b = color_b_property_->getOgreColor();
  color_util::ColorRGBA c_a(ogre_a.r, ogre_a.g, ogre_a.b, alpha_a_property_->getFloat());
  color_util::ColorRGBA c_b(ogre_b.r, ogre_b.g, ogre_b.b, alpha_b_property_->getFloat());

  unsigned int n = arrows_.size();
  SpectrumStyle style = getSpectrumStyle();
  color_util::ColorHSVA h_a, h_b;

  if (style != SpectrumStyle::rgb)
  {
    h_a = color_util::changeColorspace(c_a);
    h_b = color_util::changeColorspace(c_b);
  }

  for (unsigned int i = 0; i < n; ++i)
  {
    auto& arrow = arrows_[i];
    double v = static_cast<double>(i) / (n - 1);
    color_util::ColorRGBA spec_color;
    if (style == SpectrumStyle::rgb)
    {
      spec_color = color_util::rgbaBlend(c_a, c_b, v);
    }
    else
    {
      color_util::ColorHSVA spec_hsv;
      if (style == SpectrumStyle::hsv)
        spec_hsv = color_util::hueBlend(h_a, h_b, v);
      else
        spec_hsv = color_util::hueBlendPlus(h_a, h_b, v);
      spec_color = color_util::changeColorspace(spec_hsv);
    }
    arrow->setColor(spec_color.r, spec_color.g, spec_color.b, spec_color.a);
  }
  context_->queueRender();
}

void SpectrumDisplay::updateSize()
{
  // Read options
  unsigned int size = static_cast<unsigned int>(size_property_->getInt());
  if (size > arrows_.size())
  {
    for (size_t i = arrows_.size(); i < size; i++)
    {
      rviz::Arrow* arrow = new rviz::Arrow(scene_manager_, scene_node_);
      arrows_.push_back(arrow);
    }
  }
  else if (size < arrows_.size())
  {
    int size_signed = static_cast<int>(size);
    for (int i = arrows_.size() - 1; size_signed <= i; i--)
    {
      delete arrows_[i];
    }
    arrows_.resize(size);
  }

  for (unsigned int i = 0; i < size; ++i)
  {
    auto& arrow = arrows_[i];
    // shaft_length, shaft_diameter, head_length, head_diameter
    arrow->set(1.0, 1.0 / size, 0.0, 1.0 / size);
    arrow->setPosition(Ogre::Vector3(0.0, static_cast<double>(i) / (size - 1), 0.0));
    arrow->setDirection(Ogre::Vector3(1, 0, 0));
  }
  updateColors();
}

}  // namespace robot_nav_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::SpectrumDisplay, rviz::Display)
