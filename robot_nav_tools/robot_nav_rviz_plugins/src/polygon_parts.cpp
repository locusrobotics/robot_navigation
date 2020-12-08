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

#include <robot_nav_rviz_plugins/polygon_parts.h>
#include <nav_2d_utils/polygons.h>
#include <rviz/uniform_string_stream.h>
#include <string>
#include <vector>

namespace robot_nav_rviz_plugins
{
std_msgs::ColorRGBA getColor(rviz::ColorProperty* color_property, rviz::FloatProperty* alpha_property)
{
  const QColor& qcolor = color_property->getColor();
  std_msgs::ColorRGBA color;
  color.r = qcolor.redF();
  color.g = qcolor.greenF();
  color.b = qcolor.blueF();
  color.a = (alpha_property == nullptr) ? 1.0 : alpha_property->getFloat();
  return color;
}

PolygonOutline::PolygonOutline(Ogre::SceneManager& scene_manager, Ogre::SceneNode& scene_node)
  : scene_manager_(scene_manager), scene_node_(scene_node)
{
  manual_object_ = scene_manager_.createManualObject();
  manual_object_->setDynamic(true);
  scene_node_.attachObject(manual_object_);
}

PolygonOutline::~PolygonOutline()
{
  scene_manager_.destroyManualObject(manual_object_);
}

void PolygonOutline::reset()
{
  manual_object_->clear();
}

void PolygonOutline::setPolygon(const nav_2d_msgs::Polygon2D& polygon, const Ogre::ColourValue& color, double z_offset)
{
  manual_object_->estimateVertexCount(polygon.points.size());
  manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
  for (const nav_2d_msgs::Point2D& msg_point : polygon.points)
  {
    manual_object_->position(msg_point.x, msg_point.y, z_offset);
    manual_object_->colour(color);
  }
  manual_object_->end();
}

PolygonFill::PolygonFill(Ogre::SceneManager& scene_manager, Ogre::SceneNode& scene_node,
                         const std::string& material_name)
  : scene_manager_(scene_manager), scene_node_(scene_node), material_name_(material_name)
{
  manual_object_ = scene_manager_.createManualObject();
  manual_object_->setDynamic(true);
  scene_node_.attachObject(manual_object_);
}

PolygonFill::~PolygonFill()
{
  scene_manager_.destroyManualObject(manual_object_);
}

void PolygonFill::reset()
{
  manual_object_->clear();
  last_vertex_count_ = 0;
}


void PolygonFill::setPolygon(const nav_2d_msgs::Polygon2D& polygon, const std_msgs::ColorRGBA& color, double z_offset)
{
  nav_2d_msgs::ComplexPolygon2D complex;
  complex.outer = polygon;
  setPolygon(complex, color, z_offset);
}

void PolygonFill::setPolygon(const nav_2d_msgs::ComplexPolygon2D& polygon, const std_msgs::ColorRGBA& color,
                             double z_offset)
{
  std::vector<nav_2d_msgs::Point2D> vertices = nav_2d_utils::triangulate(polygon);
  if (vertices.empty())
  {
    return;
  }
  unsigned int vertex_count = vertices.size();

  // Based on https://github.com/ros-visualization/rviz/blob/6bf59755eb213afa575e219feb152c0efd8b3209/src/rviz/default_plugin/markers/triangle_list_marker.cpp#L113
  // If we have the same number of tris as previously, just update the object
  if (vertex_count == last_vertex_count_)
  {
    manual_object_->beginUpdate(0);
  }
  else  // Otherwise clear it and begin anew
  {
    manual_object_->clear();
    manual_object_->estimateVertexCount(vertex_count);
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    last_vertex_count_ = vertex_count;
  }

  for (unsigned int i = 0; i < vertex_count; i += 3)
  {
    std::vector<Ogre::Vector3> corners(3);
    for (size_t c = 0; c < 3; c++)
    {
      corners[c] = Ogre::Vector3(vertices[i + c].x, vertices[i + c].y, z_offset);
    }
    Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
    normal.normalise();

    for (size_t c = 0; c < 3; c++)
    {
      manual_object_->position(corners[c]);
      manual_object_->normal(normal);
      manual_object_->colour(color.r, color.g, color.b, color.a);
    }
  }
  manual_object_->end();
}

PolygonMaterial::PolygonMaterial()
{
  static uint32_t count = 0;
  rviz::UniformStringStream ss;
  ss << "PolygonMaterial" << count++;
  name_ = ss.str();
  material_ = Ogre::MaterialManager::getSingleton().create(name_, "rviz");
  material_->setReceiveShadows(false);
  material_->setCullingMode(Ogre::CULL_NONE);

  Ogre::Technique* technique = material_->getTechnique(0);
  technique->setLightingEnabled(false);
  technique->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  technique->setDepthWriteEnabled(false);
}

PolygonMaterial::~PolygonMaterial()
{
  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(name_);
}

PolygonDisplayModeProperty::PolygonDisplayModeProperty(rviz::Property* parent, const char *changed_slot)
{
  property_ = new rviz::EnumProperty("Display Mode", "Filled Outline",
       "Draw the outline, the filled-in polygon, or both", parent, changed_slot);
  property_->addOption("Outline", static_cast<int>(DisplayMode::OUTLINE));
  property_->addOption("Filled", static_cast<int>(DisplayMode::FILLED));
  property_->addOption("Filled Outline", static_cast<int>(DisplayMode::FILLED_OUTLINE));
}


}  // namespace robot_nav_rviz_plugins
