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
#include <gtest/gtest.h>
#include <dwb_plugins/simple_goal_checker.h>
#include <dwb_plugins/stopped_goal_checker.h>

using dwb_plugins::SimpleGoalChecker;
using dwb_plugins::StoppedGoalChecker;

void checkMacro(dwb_local_planner::GoalChecker& gc,
                double x0, double y0, double theta0,
                double x1, double y1, double theta1,
                double xv, double yv, double thetav,
                bool expected_result)
{
  gc.reset();
  geometry_msgs::Pose2D pose0, pose1;
  pose0.x = x0;
  pose0.y = y0;
  pose0.theta = theta0;
  pose1.x = x1;
  pose1.y = y1;
  pose1.theta = theta1;
  nav_2d_msgs::Twist2D v;
  v.x = xv;
  v.y = yv;
  v.theta = thetav;
  if (expected_result)
    EXPECT_TRUE(gc.isGoalReached(pose0, pose1, v));
  else
    EXPECT_FALSE(gc.isGoalReached(pose0, pose1, v));
}

void sameResult(dwb_local_planner::GoalChecker& gc0, dwb_local_planner::GoalChecker& gc1,
                double x0, double y0, double theta0,
                double x1, double y1, double theta1,
                double xv, double yv, double thetav,
                bool expected_result)
{
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
}

void trueFalse(dwb_local_planner::GoalChecker& gc0, dwb_local_planner::GoalChecker& gc1,
               double x0, double y0, double theta0,
               double x1, double y1, double theta1,
               double xv, double yv, double thetav)
{
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, true);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, false);
}

TEST(VelocityIterator, two_checks)
{
  ros::NodeHandle x;
  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  gc.initialize(x);
  sgc.initialize(x);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 0, true);
  sameResult(gc, sgc, 0, 0, 0, 1, 0, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 1, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 1, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, true);
  trueFalse(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 1, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_checker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
