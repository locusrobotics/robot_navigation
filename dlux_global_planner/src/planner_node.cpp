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

#include <ros/ros.h>
#include <dlux_global_planner/dlux_global_planner.h>
#include <nav_core2/exceptions.h>
#include <nav_2d_utils/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <string>

namespace dlux_global_planner
{
/**
 * @class PlannerNode
 * @brief Demonstration/debug tool that creates paths between arbitrary points.
 *
 * This node will
 *    * load a costmap from the `global_costmap` namespace
 *    * initialize the dlux_global_planner from the `planner` namespace
 *    * listen on the `/initialpose` and `/move_base_simple/goal` topics for the start and goal poses respectively
 *    * run the global planner between those poses
 *    * publish the plan as a Path and as Markers
 *
 * You can set the color of the markers with the red/green/blue parameters and you can set their namespace with
 * the marker_ns parameter.
 */
class PlannerNode
{
public:
  PlannerNode() : costmap_loader_("nav_core2", "nav_core2::Costmap"), has_start_(false), has_goal_(false)
  {
    ros::NodeHandle nh("~");
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    tf_ = std::make_shared<tf2_ros::Buffer>();
    tf2_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    std::string costmap_class;
    nh.param("global_costmap_class", costmap_class, std::string("nav_core_adapter::CostmapAdapter"));
    costmap_ = costmap_loader_.createUniqueInstance(costmap_class);
    costmap_->initialize(nh, std::string("global_costmap"), tf_);

    gp_.initialize(nh, "planner", tf_, costmap_);
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,
                                                         boost::bind(&PlannerNode::goalCB, this, _1));
    pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
                                                         boost::bind(&PlannerNode::poseCB, this, _1));
    nh.param("red", red_, 1.0);
    nh.param("green", green_, 1.0);
    nh.param("blue", blue_, 1.0);
    nh.param("marker_ns", marker_ns_, std::string(""));
  }

  ~PlannerNode()
  {
    costmap_.reset();
  }
private:
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    has_goal_ = true;
    goal_ = nav_2d_utils::poseStampedToPose2D(*goal);
    publishPointMarker(goal_, false);
    plan();
  }

  void poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal)
  {
    has_start_ = true;
    start_.header = goal->header;
    start_.pose.x = goal->pose.pose.position.x;
    start_.pose.y = goal->pose.pose.position.y;
    publishPointMarker(start_, true);
    plan();
  }

  void plan()
  {
    if (!has_goal_ || !has_start_) return;
    nav_2d_msgs::Path2D plan;
    try
    {
      plan = gp_.makePlan(start_, goal_);
    }
    catch (nav_core2::PlannerException& e)
    {
      ROS_ERROR("%s", e.what());
    }

    // publish plan as markers
    nav_msgs::Path path = nav_2d_utils::pathToPath(plan);
    double resolution = costmap_->getResolution();

    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = plan.header.frame_id;
    m.ns = marker_ns_ + "path";
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = resolution / 4;
    m.color.r = red_;
    m.color.g = green_;
    m.color.b = blue_;
    m.color.a = 0.7;
    for (unsigned int i = 0; i < plan.poses.size(); i++)
    {
      m.points.push_back(path.poses[i].pose.position);
    }
    marker_pub_.publish(m);

    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.id += 1;
    m.color.r = red_ / 2;
    m.color.g = green_ / 2;
    m.color.b = blue_ / 2;
    m.scale.x = resolution / 2;
    m.scale.y = resolution / 2;
    m.scale.z = resolution / 2;
    marker_pub_.publish(m);
  }

  void publishPointMarker(nav_2d_msgs::Pose2DStamped pose, bool start)
  {
    visualization_msgs::Marker m;
    m.header = pose.header;
    m.header.stamp = ros::Time::now();
    m.ns = (start ? "start" : "goal");
    m.type = visualization_msgs::Marker::CYLINDER;
    m.pose.position.x = pose.pose.x;
    m.pose.position.y = pose.pose.y;

    double resolution = costmap_->getResolution();
    m.scale.x = 4 * resolution;
    m.scale.y = 4 * resolution;
    m.scale.z = 0.1;
    m.color.r = start ? 0.0 : 1.0;
    m.color.g = start ? 1.0 : 0.0;
    m.color.a = 0.5;
    marker_pub_.publish(m);
  }

  ros::Subscriber goal_sub_, pose_sub_;
  ros::Publisher marker_pub_;

  TFListenerPtr tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_;
  nav_core2::Costmap::Ptr costmap_;
  pluginlib::ClassLoader<nav_core2::Costmap> costmap_loader_;
  dlux_global_planner::DluxGlobalPlanner gp_;

  nav_2d_msgs::Pose2DStamped start_, goal_;
  bool has_start_, has_goal_;

  double red_, green_, blue_;
  std::string marker_ns_;
};
}  // namespace dlux_global_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");
  dlux_global_planner::PlannerNode pn;
  ros::spin();
  return 0;
}
