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

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <nav_grid_server/image_loader.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <nav_grid_pub_sub/nav_grid_publisher.h>
#include <nav_grid_pub_sub/occ_grid_message_utils.h>
#include <yaml-cpp/yaml.h>
#include <string>

class MapServer
{
public:
  explicit MapServer(const std::string& filename) : pub_(the_map_)
  {
    ros::NodeHandle global_nh, private_nh("~");

    std::string image_filename;

    // The following parameters will be overwritten by the ROS parameters (if they exist), or the yaml parameters
    std::string frame_id = "map";
    double resolution = 0.05;
    bool negate = false;
    double occ_th = 0.65;
    double free_th = 0.196;
    std::string mode = "trinary";
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::string occupancy_grid_topic = "map";
    std::string nav_grid_topic = "static_map";

    private_nh.param("frame_id", frame_id, frame_id);

    if (filename.find("yaml") != std::string::npos)
    {
      YAML::Node config = YAML::LoadFile(filename.c_str());
      image_filename = config["image"].as<std::string>();
      if (image_filename[0] != '/')
      {
        boost::filesystem::path yaml_path(filename);
        boost::filesystem::path image_path = yaml_path.parent_path() / image_filename;
        image_filename = image_path.string();
      }

      if (config["resolution"]) resolution = config["resolution"].as<double>();
      // Allow for either boolean or integer (0|1)
      if (config["negate"]) negate = config["negate"].as<bool, int>(0);
      if (config["occupied_thresh"]) occ_th = config["occupied_thresh"].as<double>();
      if (config["free_thresh"]) free_th = config["free_thresh"].as<double>();
      if (config["mode"]) mode = config["mode"].as<std::string>();
      if (config["origin"])
      {
        origin_x = config["origin"][0].as<double>();
        origin_y = config["origin"][1].as<double>();
      }
    }
    else
    {
      image_filename = filename;
    }

    // If the parameter exists, use it, otherwise use the previous value
    private_nh.param("resolution", resolution, resolution);
    private_nh.param("negate", negate, negate);
    private_nh.param("occupied_thresh", occ_th, occ_th);
    private_nh.param("free_thresh", free_th, free_th);
    private_nh.param("mode", mode, mode);
    private_nh.param("origin_x", origin_x, origin_x);
    private_nh.param("origin_y", origin_y, origin_y);
    private_nh.param("occupancy_grid_topic", occupancy_grid_topic, occupancy_grid_topic);
    private_nh.param("nav_grid_topic", nav_grid_topic, nav_grid_topic);

    the_map_ = nav_grid_server::classicLoadMapFromFile(image_filename, resolution, negate, occ_th, free_th, mode);
    nav_grid::NavGridInfo full_info = the_map_.getInfo();
    full_info.origin_x = origin_x;
    full_info.origin_y = origin_y;
    the_map_.setInfo(full_info);

    // To make sure get a consistent time in simulation
    ros::Time::waitForValid();
    ROS_INFO_NAMED("MapServer", "Read a %d X %d map @ %.3lf m/cell",
                   the_map_.getWidth(), the_map_.getHeight(), the_map_.getResolution());

    grid_version_ = nav_grid_pub_sub::toOccupancyGrid(the_map_, ros::Time::now());

    pub_.init(global_nh, nav_grid_topic, occupancy_grid_topic, "", false);
    pub_.setCostInterpretation(nav_grid_pub_sub::RAW);
    pub_.publish();

    // Service provider
    service_ = global_nh.advertiseService("static_map", &MapServer::mapCallback, this);

    // Latched publisher for metadata
    metadata_pub_ = global_nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    metadata_pub_.publish(grid_version_.info);
  }

private:
  nav_grid_pub_sub::NavGridPublisher pub_;
  ros::Publisher metadata_pub_;
  ros::ServiceServer service_;

  nav_grid::VectorNavGrid<unsigned char> the_map_;
  nav_msgs::OccupancyGrid grid_version_;

  /** Callback invoked when someone requests our service */
  bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
  {
    // request is empty; we ignore it
    res.map = grid_version_;
    ROS_INFO_NAMED("MapServer", "Sending map");

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if (argc != 2)
  {
    ROS_ERROR_NAMED("MapServer", "USAGE: server (map.yaml|map_image)");
    ROS_ERROR_NAMED("MapServer", "  map.yaml: map description file");
    ROS_ERROR_NAMED("MapServer", "  map_image: any valid image file");
    exit(EXIT_FAILURE);
  }

  try
  {
    MapServer ms(argv[1]);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR_NAMED("MapServer", "exception: %s", e.what());
    exit(EXIT_FAILURE);
  }

  return 0;
}
