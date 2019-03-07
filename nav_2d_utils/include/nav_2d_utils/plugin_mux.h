/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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

#ifndef NAV_2D_UTILS_PLUGIN_MUX_H
#define NAV_2D_UTILS_PLUGIN_MUX_H

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>
#include <nav_2d_msgs/SwitchPlugin.h>
#include <map>
#include <string>
#include <vector>

namespace nav_2d_utils
{
/**
 * @class PluginMux
 * @brief An organizer for switching between multiple different plugins of the same type
 *
 * The different plugins are specified using a list of strings on the parameter server, each of which is a namespace.
 * The specific type and additional parameters for each plugin are specified on the parameter server in that namespace.
 * All the plugins are loaded initially, but only one is the "current" plugin at any particular time, which is published
 * on a latched topic AND stored on the ROS parameter server. You can switch which plugin is current either through a
 * C++ or ROS interface.
 */
template<class T>
class PluginMux
{
public:
  /**
   * @brief Main Constructor - Loads plugin(s) and sets up ROS interfaces
   *
   * @param plugin_package The package of the plugin type
   * @param plugin_class The class name for the plugin type
   * @param parameter_name Name of parameter for the namespaces.
   * @param default_value If class name is not specified, which plugin should be loaded
   * @param ros_name ROS name for setting up topic and parameter
   * @param switch_service_name ROS name for setting up the ROS service
   */
  PluginMux(const std::string& plugin_package, const std::string& plugin_class,
            const std::string& parameter_name, const std::string& default_value,
            const std::string& ros_name = "current_plugin", const std::string& switch_service_name = "switch_plugin");

  /**
   * @brief Create an instance of the given plugin_class_name and save it with the given plugin_name
   * @param plugin_name Namespace for the new plugin
   * @param plugin_class_name Class type name for the new plugin
   */
  void addPlugin(const std::string& plugin_name, const std::string& plugin_class_name);

  /**
   * @brief C++ Interface for switching to a given plugin
   *
   * @param name Namespace to set current plugin to
   * @return true if that namespace exists and is loaded properly
   */
  bool usePlugin(const std::string& name)
  {
    // If plugin with given name doesn't exist, return false
    if (plugins_.count(name) == 0) return false;

    if (switch_callback_)
    {
      switch_callback_(current_plugin_, name);
    }

    // Switch Mux
    current_plugin_ = name;

    // Update ROS info
    std_msgs::String str_msg;
    str_msg.data = current_plugin_;
    current_plugin_pub_.publish(str_msg);
    private_nh_.setParam(ros_name_, current_plugin_);

    return true;
  }

  /**
   * @brief Get the Current Plugin Name
   * @return Name of the current plugin
   */
  std::string getCurrentPluginName() const
  {
    return current_plugin_;
  }

  /**
   * @brief Check to see if a plugin exists
   * @param name Namespace to set current plugin to
   * @return True if the plugin exists
   */
  bool hasPlugin(const std::string& name) const
  {
    return plugins_.find(name) != plugins_.end();
  }

  /**
   * @brief Get the Specified Plugin
   * @param name Name of plugin to get
   * @return Reference to specified plugin
   */
  T& getPlugin(const std::string& name)
  {
    if (!hasPlugin(name))
      throw std::invalid_argument("Plugin not found");
    return *plugins_[name];
  }

  /**
   * @brief Get the Current Plugin
   * @return Reference to current plugin
   */
  T& getCurrentPlugin()
  {
    return getPlugin(current_plugin_);
  }

  /**
   * @brief Get the current list of plugin names
   */
  std::vector<std::string> getPluginNames() const
  {
    std::vector<std::string> names;
    for (auto& kv : plugins_)
    {
      names.push_back(kv.first);
    }
    return names;
  }

  /**
   * @brief alias for the function-type of the callback fired when the plugin switches.
   *
   * The first parameter will be the namespace of the plugin that was previously used.
   * The second parameter will be the namespace of the plugin that is being switched to.
   */
  using SwitchCallback = std::function<void(const std::string&, const std::string&)>;

  /**
   * @brief Set the callback fired when the plugin switches
   *
   * In addition to switching which plugin is being used via directly calling `usePlugin`
   * a switch can also be triggered externally via ROS service. In such a case, other classes
   * may want to know when this happens. This is accomplished using the SwitchCallback, which
   * will be called regardless of how the plugin is switched.
   */
  void setSwitchCallback(SwitchCallback switch_callback) { switch_callback_ = switch_callback; }

protected:
  /**
   * @brief ROS Interface for Switching Plugins
   */
  bool switchPluginService(nav_2d_msgs::SwitchPlugin::Request &req, nav_2d_msgs::SwitchPlugin::Response &resp)
  {
    std::string name = req.new_plugin;
    if (usePlugin(name))
    {
      resp.success = true;
      resp.message = "Loaded plugin namespace " + name + ".";
    }
    else
    {
      resp.success = false;
      resp.message = "Namespace " + name + " not configured!";
    }
    return true;
  }

  // Plugin Management
  pluginlib::ClassLoader<T> plugin_loader_;
  std::map<std::string, boost::shared_ptr<T>> plugins_;
  std::string current_plugin_;

  // ROS Interface
  ros::ServiceServer switch_plugin_srv_;
  ros::Publisher current_plugin_pub_;
  ros::NodeHandle private_nh_;
  std::string ros_name_;

  // Switch Callback
  SwitchCallback switch_callback_;
};

// *********************************************************************************************************************
// ****************** Implementation of Templated Methods **************************************************************
// *********************************************************************************************************************
template<class T>
PluginMux<T>::PluginMux(const std::string& plugin_package, const std::string& plugin_class,
                        const std::string& parameter_name, const std::string& default_value,
                        const std::string& ros_name, const std::string& switch_service_name)
  : plugin_loader_(plugin_package, plugin_class), private_nh_("~"), ros_name_(ros_name), switch_callback_(nullptr)
{
  // Create the latched publisher
  current_plugin_pub_ = private_nh_.advertise<std_msgs::String>(ros_name_, 1, true);

  // Load Plugins
  std::string plugin_class_name;
  std::vector<std::string> plugin_namespaces;
  private_nh_.getParam(parameter_name, plugin_namespaces);
  if (plugin_namespaces.size() == 0)
  {
    // If no namespaces are listed, use the name of the default class as the singular namespace.
    std::string plugin_name = plugin_loader_.getName(default_value);
    plugin_namespaces.push_back(plugin_name);
  }

  for (const std::string& the_namespace : plugin_namespaces)
  {
    // Load the class name from namespace/plugin_class, or use default value
    private_nh_.param(std::string(the_namespace + "/plugin_class"), plugin_class_name, default_value);
    addPlugin(the_namespace, plugin_class_name);
  }

  // By default, use the first one as current
  usePlugin(plugin_namespaces[0]);

  // Now that the plugins are created, advertise the service if there are multiple
  if (plugin_namespaces.size() > 1)
  {
    switch_plugin_srv_ = private_nh_.advertiseService(switch_service_name, &PluginMux::switchPluginService, this);
  }
}

template<class T>
void PluginMux<T>::addPlugin(const std::string& plugin_name, const std::string& plugin_class_name)
{
  try
  {
    plugins_[plugin_name] = plugin_loader_.createInstance(plugin_class_name);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_NAMED("PluginMux",
                    "Failed to load the plugin: %s. Exception: %s", plugin_name.c_str(), ex.what());
    exit(EXIT_FAILURE);
  }
}

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS_PLUGIN_MUX_H
