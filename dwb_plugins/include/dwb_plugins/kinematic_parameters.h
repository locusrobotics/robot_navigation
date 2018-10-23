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

#ifndef DWB_PLUGINS_KINEMATIC_PARAMETERS_H
#define DWB_PLUGINS_KINEMATIC_PARAMETERS_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dwb_plugins/KinematicParamsConfig.h>

namespace dwb_plugins
{

/**
 * @class KinematicParameters
 * @brief A dynamically reconfigurable class containing one representation of the robot's kinematics
 */
class KinematicParameters
{
public:
  KinematicParameters();
  void initialize(const ros::NodeHandle& nh);

  inline double getMinX() { return min_vel_x_; }
  inline double getMaxX() { return max_vel_x_; }
  inline double getAccX() { return acc_lim_x_; }
  inline double getDecelX() { return decel_lim_x_; }

  inline double getMinY() { return min_vel_y_; }
  inline double getMaxY() { return max_vel_y_; }
  inline double getAccY() { return acc_lim_y_; }
  inline double getDecelY() { return decel_lim_y_; }

  inline double getMinSpeedXY() { return min_speed_xy_; }

  inline double getMinTheta() { return -max_vel_theta_; }
  inline double getMaxTheta() { return max_vel_theta_; }
  inline double getAccTheta() { return acc_lim_theta_; }
  inline double getDecelTheta() { return decel_lim_theta_; }
  inline double getMinSpeedTheta() { return min_speed_theta_; }

  /**
   * @brief Check to see whether the combined x/y/theta velocities are valid
   * @return True if the magnitude hypot(x,y) and theta are within the robot's absolute limits
   *
   * This is based on three parameters: min_speed_xy, max_speed_xy and min_speed_theta.
   * The speed is valid if
   *  1) The combined magnitude hypot(x,y) is less than max_speed_xy (or max_speed_xy is negative)
   *  AND
   *  2) min_speed_xy is negative or min_speed_theta is negative or
   *     hypot(x,y) is greater than min_speed_xy or fabs(theta) is greater than min_speed_theta.
   *
   * In English, it makes sure the diagonal motion is not too fast,
   * and that the velocity is moving in some meaningful direction.
   *
   * In Latin, quod si motus sit signum quaerit et movere ieiunium et significantissime comprehendite.
   */
  bool isValidSpeed(double x, double y, double theta);

  using Ptr = std::shared_ptr<KinematicParameters>;
protected:
  // For parameter descriptions, see cfg/KinematicParams.cfg
  double min_vel_x_, min_vel_y_;
  double max_vel_x_, max_vel_y_, max_vel_theta_;

  double min_speed_xy_, max_speed_xy_;
  double min_speed_theta_;

  double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
  double decel_lim_x_, decel_lim_y_, decel_lim_theta_;

  // Cached square values of min_speed_xy and max_speed_xy
  double min_speed_xy_sq_, max_speed_xy_sq_;

  void reconfigureCB(KinematicParamsConfig &config, uint32_t level);
  std::shared_ptr<dynamic_reconfigure::Server<KinematicParamsConfig> > dsrv_;
};

}  // namespace dwb_plugins

#endif  // DWB_PLUGINS_KINEMATIC_PARAMETERS_H
