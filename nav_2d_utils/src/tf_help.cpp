/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *                2022, Metro Robots
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

#include <nav_2d_utils/tf_help.h>
#include <nav_2d_msgs/Pose2DStamped.h>
#include <nav_2d_utils/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

namespace nav_2d_utils
{
bool transformPose(const TFListenerPtr tf, const std::string frame,
                   const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose,
                   const bool extrapolation_fallback)
{
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf->transform(in_pose, out_pose, frame);
    return true;
  }
  catch (tf::ExtrapolationException& ex)
  {
    if (!extrapolation_fallback)
      throw;
    geometry_msgs::PoseStamped latest_in_pose;
    latest_in_pose.header.frame_id = in_pose.header.frame_id;
    latest_in_pose.pose = in_pose.pose;
    tf->transform(latest_in_pose, out_pose, frame);
    return true;
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Exception in transformPose: %s", ex.what());
    return false;
  }
  return false;
}

bool transformPose(const TFListenerPtr tf, const std::string frame,
                   const nav_2d_msgs::Pose2DStamped& in_pose, nav_2d_msgs::Pose2DStamped& out_pose,
                   const bool extrapolation_fallback)
{
  geometry_msgs::PoseStamped in_3d_pose = pose2DToPoseStamped(in_pose);
  geometry_msgs::PoseStamped out_3d_pose;

  bool ret = transformPose(tf, frame, in_3d_pose, out_3d_pose, extrapolation_fallback);
  if (ret)
  {
    out_pose = poseStampedToPose2D(out_3d_pose);
  }
  return ret;
}

geometry_msgs::Pose2D transformStampedPose(const TFListenerPtr tf, const nav_2d_msgs::Pose2DStamped& pose,
                                           const std::string& frame_id)
{
  nav_2d_msgs::Pose2DStamped local_pose;
  nav_2d_utils::transformPose(tf, frame_id, pose, local_pose);
  return local_pose.pose;
}

}  // namespace nav_2d_utils
