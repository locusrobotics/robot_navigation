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
#include <ros/ros.h>
#include <nav_grid/vector_nav_grid.h>
#include <nav_grid_pub_sub/nav_grid_subscriber.h>
#include <nav_grid_pub_sub/cost_interpretation.h>
#include <nav_grid_pub_sub/cost_interpretation_tables.h>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <string>
#include <vector>

class MapSaver
{
public:
  MapSaver() : map_sub_(data_), written_once_(false)
  {
    ros::NodeHandle nh, private_nh("~");

    // Load settings from the parameter server
    std::string topic;
    bool use_nav_grid;
    private_nh.param("topic", topic, std::string("map"));
    private_nh.param("nav_grid", use_nav_grid, false);
    private_nh.param("once", once_, true);
    private_nh.param("write_unstamped", write_unstamped_, true);
    private_nh.param("write_stamped", write_stamped_, false);
    if (!write_unstamped_ && !write_stamped_)
    {
      ROS_FATAL_NAMED("MapSaver", "write_unstamped and write_stamped both set to false. Nothing will be written.");
      exit(EXIT_FAILURE);
    }

    bool trinary_output;
    private_nh.param("trinary_output", trinary_output, true);
    if (trinary_output)
    {
      output_interpretation_ = nav_grid_pub_sub::TRINARY_SAVE;
    }
    else
    {
      output_interpretation_ = nav_grid_pub_sub::SCALE_SAVE;
    }

    private_nh.param("map_extension", map_extension_, std::string("png"));
    private_nh.param("map_prefix", map_prefix_, std::string("map"));
    std::string output_directory_str;
    private_nh.param("output_directory", output_directory_str, std::string("."));
    output_directory_ = boost::filesystem::path(output_directory_str);
    if (!boost::filesystem::exists(output_directory_))
    {
      // Create the output folder
      try
      {
        boost::filesystem::create_directories(output_directory_);
      }
      catch (const std::exception& e)
      {
        ROS_FATAL_STREAM("Unable to create the requested output directory (" + output_directory_.string() + "). "
                         "Error: " + e.what());
        exit(EXIT_FAILURE);
      }
    }

    ROS_INFO_NAMED("MapSaver", "Waiting for the map");
    map_sub_.setCostInterpretation(nav_grid_pub_sub::getOccupancyInput(trinary_output, true));
    map_sub_.init(nh, std::bind(&MapSaver::newDataCallback, this, std::placeholders::_1), topic, use_nav_grid, !once_);
  }

  boost::filesystem::path generatePath(const std::string& extension, const ros::Time& stamp) const
  {
    boost::filesystem::path output;
    if (stamp == ros::Time())
    {
      output = output_directory_ / boost::filesystem::path(map_prefix_ + "." + extension);
    }
    else
    {
      std::stringstream ss;
      ss << map_prefix_ << "-" << std::setw(10) << std::setfill('0') << stamp.sec << "." << extension;
      output = output_directory_ / boost::filesystem::path(ss.str());
    }
    return output;
  }

  void newDataCallback(const nav_core2::UIntBounds& bounds)
  {
    const nav_grid::NavGridInfo& info = data_.getInfo();
    ROS_INFO_NAMED("MapSaver", "Writing a new map: %s", info.toString().c_str());

    // Convert nav_grid into an opencv image
    cv::Mat image(info.height, info.width, CV_8UC1);
    for (unsigned int row = 0; row < info.height; ++row)
    {
      unsigned char* output_row = image.ptr<unsigned char>(row);
      for (unsigned int col = 0; col < info.width; ++col)
      {
        unsigned char input = data_(col, info.height - row - 1);  // flip vertically
        output_row[col] = nav_grid_pub_sub::interpretCost(input, output_interpretation_);
      }
    }

    // Save output image and yaml
    if (write_unstamped_)
    {
      writeData(ros::Time(), image, info);
    }

    if (write_stamped_)
    {
      writeData(ros::Time::now(), image, info);
    }
    written_once_ = true;
  }

  void writeData(const ros::Time& stamp, const cv::Mat& image, const nav_grid::NavGridInfo& info) const
  {
    // Save the Image
    auto map_path = generatePath(map_extension_, stamp);
    cv::imwrite(map_path.string(), image);

    // Save the yaml file
    auto yaml_path = generatePath("yaml", stamp);
    std::ofstream yaml_file(yaml_path.c_str(), std::ofstream::out);
    if (yaml_file.is_open())
    {
      yaml_file << "image: " << map_path.filename().string() << "\n";
      yaml_file << "resolution: " << info.resolution << "\n";
      yaml_file << "origin: [" << info.origin_x << ", " << info.origin_y << ", 0]\n";
      yaml_file << "occupied_thresh: 0.65\n";
      yaml_file << "free_thresh: 0.196\n";
      yaml_file << "negate: 0\n";
      yaml_file.close();
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(10.0, "Could not open output file '" + yaml_path.string() + "'");
    }
  }

  void spin()
  {
    // Keep spinning unless we just want to run once and we wrote our map once
    while (ros::ok() && !(once_ && written_once_))
      ros::spinOnce();
  }

protected:
  nav_grid_pub_sub::NavGridSubscriber map_sub_;
  nav_grid::VectorNavGrid<unsigned char> data_;
  std::string map_extension_, map_prefix_;
  boost::filesystem::path output_directory_;
  bool once_, write_stamped_, write_unstamped_, written_once_;
  std::vector<unsigned char> output_interpretation_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  MapSaver ms;
  ms.spin();
  return 0;
}
