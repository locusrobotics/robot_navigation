/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
#include <dwb_local_planner/publisher.h>
#include <nav_grid/coordinate_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_2d_utils/conversions.h>
#include <memory>
#include <vector>

namespace dwb_local_planner
{

void DWBPublisher::initialize(ros::NodeHandle& nh)
{
  ros::NodeHandle global_nh;
  // Load Publishers
  nh.param("publish_evaluation", publish_evaluation_, true);
  if (publish_evaluation_)
    eval_pub_ = nh.advertise<dwb_msgs::LocalPlanEvaluation>("evaluation", 1);

  nh.param("publish_input_params", publish_input_params_, true);
  if (publish_input_params_)
  {
    info_pub_ = nh.advertise<nav_2d_msgs::NavGridInfo>("info", 1);
    pose_pub_ = nh.advertise<geometry_msgs::Pose2D>("pose", 1);
    goal_pub_ = nh.advertise<geometry_msgs::Pose2D>("goal", 1);
    velocity_pub_ = nh.advertise<nav_2d_msgs::Twist2D>("velocity", 1);
  }

  nh.param("publish_global_plan", publish_global_plan_, true);
  if (publish_global_plan_)
    global_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);

  nh.param("publish_transformed_plan", publish_transformed_, true);
  if (publish_transformed_)
    transformed_pub_ = nh.advertise<nav_msgs::Path>("transformed_global_plan", 1);

  nh.param("publish_local_plan", publish_local_plan_, true);
  if (publish_local_plan_)
    local_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);

  nh.param("publish_trajectories", publish_trajectories_, true);
  if (publish_trajectories_)
    marker_pub_ = global_nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
  double marker_lifetime;
  nh.param("marker_lifetime", marker_lifetime, 0.1);
  marker_lifetime_ = ros::Duration(marker_lifetime);

  nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
  if (publish_cost_grid_pc_)
    cost_grid_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cost_cloud", 1);
}

void DWBPublisher::publishEvaluation(std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results)
{
  if (results == nullptr) return;
  if (publish_evaluation_ && eval_pub_.getNumSubscribers() > 0)
  {
    eval_pub_.publish(*results);
  }
  publishTrajectories(*results);
}

void DWBPublisher::publishTrajectories(const dwb_msgs::LocalPlanEvaluation& results)
{
  if (!publish_trajectories_ || marker_pub_.getNumSubscribers() == 0) return;
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker m;

  if (results.twists.size() == 0) return;

  geometry_msgs::Point pt;

  m.header = results.header;
  m.type = m.LINE_STRIP;
  m.pose.orientation.w = 1;
  m.scale.x = 0.002;
  m.color.a = 1.0;
  m.lifetime = marker_lifetime_;

  double best_cost = results.twists[results.best_index].total,
         worst_cost = results.twists[results.worst_index].total,
         denominator = worst_cost - best_cost;
  if (std::fabs(denominator) < 1e-9)
  {
    denominator = 1.0;
  }

  for (unsigned int i = 0; i < results.twists.size(); i++)
  {
    const dwb_msgs::TrajectoryScore& twist = results.twists[i];
    if (twist.total >= 0)
    {
      m.color.r = 1 - (twist.total - best_cost) / denominator;
      m.color.g = 1 - (twist.total - best_cost) / denominator;
      m.color.b = 1;
      m.ns = "ValidTrajectories";
    }
    else
    {
      m.color.b = 0;
      m.ns = "InvalidTrajectories";
    }
    m.points.clear();
    for (unsigned int j = 0; j < twist.traj.poses.size(); ++j)
    {
      pt.x = twist.traj.poses[j].x;
      pt.y = twist.traj.poses[j].y;
      pt.z = 0;
      m.points.push_back(pt);
    }
    ma.markers.push_back(m);
    m.id += 1;
  }

  marker_pub_.publish(ma);
}

void DWBPublisher::publishLocalPlan(const std_msgs::Header& header,
                                    const dwb_msgs::Trajectory2D& traj)
{
  if (!publish_local_plan_ || local_pub_.getNumSubscribers() == 0) return;

  nav_msgs::Path path = nav_2d_utils::poses2DToPath(traj.poses, header.frame_id, header.stamp);
  local_pub_.publish(path);
}

void DWBPublisher::publishCostGrid(const nav_core2::Costmap::Ptr costmap,
                                   const std::vector<TrajectoryCritic::Ptr> critics)
{
  if (!publish_cost_grid_pc_ || cost_grid_pc_pub_.getNumSubscribers() == 0) return;

  const nav_grid::NavGridInfo& info = costmap->getInfo();
  sensor_msgs::PointCloud cost_grid_pc;
  cost_grid_pc.header.frame_id = info.frame_id;
  cost_grid_pc.header.stamp = ros::Time::now();

  double x_coord, y_coord;
  unsigned int n = info.width * info.height;
  cost_grid_pc.points.resize(n);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < info.height; cy++)
  {
    for (unsigned int cx = 0; cx < info.width; cx++)
    {
      gridToWorld(info, cx, cy, x_coord, y_coord);
      cost_grid_pc.points[i].x = x_coord;
      cost_grid_pc.points[i].y = y_coord;
      i++;
    }
  }

  sensor_msgs::ChannelFloat32 totals;
  totals.name = "total_cost";
  totals.values.resize(n);

  for (TrajectoryCritic::Ptr critic : critics)
  {
    unsigned int channel_index = cost_grid_pc.channels.size();
    critic->addCriticVisualization(cost_grid_pc);
    if (channel_index == cost_grid_pc.channels.size())
    {
      // No channels were added, so skip to next critic
      continue;
    }
    double scale = critic->getScale();
    for (i = 0; i < n; i++)
    {
      totals.values[i] += cost_grid_pc.channels[channel_index].values[i] * scale;
    }
  }
  cost_grid_pc.channels.push_back(totals);

  sensor_msgs::PointCloud2 cost_grid_pc2;
  convertPointCloudToPointCloud2(cost_grid_pc, cost_grid_pc2);
  cost_grid_pc_pub_.publish(cost_grid_pc2);
}

void DWBPublisher::publishGlobalPlan(const nav_2d_msgs::Path2D plan)
{
  publishGenericPlan(plan, global_pub_, publish_global_plan_);
}

void DWBPublisher::publishTransformedPlan(const nav_2d_msgs::Path2D plan)
{
  publishGenericPlan(plan, transformed_pub_, publish_transformed_);
}

void DWBPublisher::publishLocalPlan(const nav_2d_msgs::Path2D plan)
{
  publishGenericPlan(plan, local_pub_, publish_local_plan_);
}

void DWBPublisher::publishGenericPlan(const nav_2d_msgs::Path2D plan, const ros::Publisher pub, bool flag)
{
  if (!flag || pub.getNumSubscribers() == 0) return;
  nav_msgs::Path path = nav_2d_utils::pathToPath(plan);
  pub.publish(path);
}

void DWBPublisher::publishInputParams(const nav_grid::NavGridInfo& info, const geometry_msgs::Pose2D& start_pose,
                                      const nav_2d_msgs::Twist2D& velocity, const geometry_msgs::Pose2D& goal_pose)
{
  if (!publish_input_params_) return;

  info_pub_.publish(nav_2d_utils::toMsg(info));
  pose_pub_.publish(start_pose);
  goal_pub_.publish(goal_pose);
  velocity_pub_.publish(velocity);
}

}  // namespace dwb_local_planner
