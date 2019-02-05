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
#include <gtest/gtest.h>
#include <nav_core_adapter/local_planner_adapter.h>


TEST(LocalPlannerAdapter, unload_local_planner)
{
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);

  // This empty transform is added to satisfy the constructor of
  // Costmap2DROS, which waits for the transform from map to base_link
  // to become available.
  geometry_msgs::TransformStamped base_rel_map;
  base_rel_map.child_frame_id = "/base_link";
  base_rel_map.header.frame_id = "/map";
  base_rel_map.transform.rotation.w = 1.0;
  tf.setTransform(base_rel_map, "unload", true);

  nav_core_adapter::LocalPlannerAdapter* lpa = new nav_core_adapter::LocalPlannerAdapter();

  costmap_2d::Costmap2DROS costmap_ros("local_costmap", tf);
  lpa->initialize("lpa", &tf, &costmap_ros);

  delete lpa;

  // Simple test to make sure costmap hasn't been deleted
  EXPECT_EQ("map", costmap_ros.getGlobalFrameID());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "unload_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
