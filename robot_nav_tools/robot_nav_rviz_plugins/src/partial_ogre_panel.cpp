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
#include <string>
#include <vector>

namespace robot_nav_rviz_plugins
{
OgrePanel::PartialOgrePanel::PartialOgrePanel(Ogre::SceneManager& scene_manager,
                                              Ogre::SceneNode& parent_scene_node,
                                              const nav_core2::UIntBounds& bounds,
                                              float resolution)
  : scene_manager_(scene_manager), manual_object_(NULL), bounds_(bounds)
{
  // Set up map material
  static int material_count = 0;
  std::stringstream material_name;
  material_name << "NavGridMaterial" << material_count++;
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone(material_name.str());

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);

  static int map_count = 0;
  std::stringstream object_name;
  object_name << "NavGridObject" << map_count++;
  manual_object_ = scene_manager_.createManualObject(object_name.str());

  static int node_count = 0;
  std::stringstream node_name;
  node_name << "NGNodeObject" << node_count++;

  scene_node_ = parent_scene_node.createChildSceneNode(node_name.str());
  scene_node_->attachObject(manual_object_);

  /*
   * Divide the rectangular panel into two large triangles for rendering in Ogre
   * o-----o
   * |    /|
   * |   / |
   * |  /  |
   * | /   |
   * |/    |
   * o-----o
   */
  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // First triangle - Bottom left
  manual_object_->position(0.0f, 0.0f, 0.0f);
  manual_object_->textureCoord(0.0f, 0.0f);
  manual_object_->normal(0.0f, 0.0f, 1.0f);

  // First triangle - Top right
  manual_object_->position(1.0f, 1.0f, 0.0f);
  manual_object_->textureCoord(1.0f, 1.0f);
  manual_object_->normal(0.0f, 0.0f, 1.0f);

  // First triangle - Top left
  manual_object_->position(0.0f, 1.0f, 0.0f);
  manual_object_->textureCoord(0.0f, 1.0f);
  manual_object_->normal(0.0f, 0.0f, 1.0f);

  // Second triangle - Bottom left
  manual_object_->position(0.0f, 0.0f, 0.0f);
  manual_object_->textureCoord(0.0f, 0.0f);
  manual_object_->normal(0.0f, 0.0f, 1.0f);

  // Second triangle - Bottom right
  manual_object_->position(1.0f, 0.0f, 0.0f);
  manual_object_->textureCoord(1.0f, 0.0f);
  manual_object_->normal(0.0f, 0.0f, 1.0f);

  // Second triangle - Top right
  manual_object_->position(1.0f, 1.0f, 0.0f);
  manual_object_->textureCoord(1.0f, 1.0f);
  manual_object_->normal(0.0f, 0.0f, 1.0f);

  manual_object_->end();

  scene_node_->setPosition(bounds_.getMinX() * resolution, bounds_.getMinY() * resolution, 0);
  scene_node_->setScale(bounds_.getWidth() * resolution, bounds_.getHeight() * resolution, 1.0);

  // Set default data (crucial so that it fails on construction if too big)
  std::vector<unsigned char> pixels;
  pixels.resize(bounds_.getWidth() * bounds_.getHeight());
  updateData(pixels);

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible(false);
}

OgrePanel::PartialOgrePanel::~PartialOgrePanel()
{
  manual_object_->detachFromParent();
  scene_manager_.destroyManualObject(manual_object_);
}

void OgrePanel::PartialOgrePanel::updateData(std::vector<unsigned char>& pixels)
{
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(&pixels[0], pixels.size()));

  if (!texture_.isNull())
  {
    Ogre::TextureManager::getSingleton().remove(texture_->getName());
    texture_.setNull();
  }

  static int tex_count = 0;
  std::stringstream texture_name;
  texture_name << "MapTexture" << tex_count++;
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(texture_name.str(),
             Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
             pixel_stream, bounds_.getWidth(), bounds_.getHeight(), Ogre::PF_L8, Ogre::TEX_TYPE_2D,
             0);

  setTexture(texture_->getName(), 0);
  manual_object_->setVisible(true);
}

void OgrePanel::PartialOgrePanel::clear()
{
  if (manual_object_)
    manual_object_->setVisible(false);

  if (!texture_.isNull())
  {
    Ogre::TextureManager::getSingleton().remove(texture_->getName());
    texture_.setNull();
  }
}

void OgrePanel::PartialOgrePanel::setTexture(const std::string& texture_name, int index)
{
  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > index)
  {
    tex_unit = pass->getTextureUnitState(index);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }
  tex_unit->setTextureName(texture_name);
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);
}

void OgrePanel::PartialOgrePanel::updateAlphaRendering(Ogre::SceneBlendType scene_blending, bool depth_write, int group,
                                                       Ogre::Renderable::Visitor* alpha_setter)
{
  material_->setSceneBlending(scene_blending);
  material_->setDepthWriteEnabled(depth_write);
  if (manual_object_)
  {
    manual_object_->visitRenderables(alpha_setter);
    manual_object_->setRenderQueueGroup(group);
  }
}

}  // namespace robot_nav_rviz_plugins
