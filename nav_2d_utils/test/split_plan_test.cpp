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
#include <gtest/gtest.h>
#include <nav_2d_utils/path_ops.h>
#include <vector>

using nav_2d_utils::splitPlan;
using nav_2d_utils::addPose;

TEST(SplitPlanTest, split_plan_test)
{
  nav_2d_msgs::Path2D path;
  // ======= segment 0 =========
  addPose(path, 8.975, 9.225, 0.7854);
  addPose(path, 8.9694, 9.2194, 0.7854);
  addPose(path, 8.9639, 9.2139, 0.7854);
  addPose(path, 8.9583, 9.2083, 0.7854);
  addPose(path, 8.9528, 9.2028, 0.7854);
  addPose(path, 8.9472, 9.1972, 0.7854);
  addPose(path, 8.9417, 9.1917, 0.7854);
  addPose(path, 8.9361, 9.1861, 0.7854);
  addPose(path, 8.9306, 9.1806, 0.7854);
  addPose(path, 8.925, 9.175, 0.7854);
  addPose(path, 8.9194, 9.1694, 0.7854);
  addPose(path, 8.9139, 9.1639, 0.7854);
  addPose(path, 8.9083, 9.1583, 0.7854);
  addPose(path, 8.9028, 9.1528, 0.7854);
  addPose(path, 8.8972, 9.1472, 0.7854);
  addPose(path, 8.8917, 9.1417, 0.7854);
  addPose(path, 8.8861, 9.1361, 0.7854);
  addPose(path, 8.8806, 9.1306, 0.7854);
  addPose(path, 8.875, 9.125, 0.7854);
  addPose(path, 8.8694, 9.1194, 0.7854);
  addPose(path, 8.8639, 9.1139, 0.7854);
  addPose(path, 8.8583, 9.1083, 0.7854);
  addPose(path, 8.8528, 9.1028, 0.7854);
  addPose(path, 8.8472, 9.0972, 0.7854);
  addPose(path, 8.8417, 9.0917, 0.7854);
  addPose(path, 8.8361, 9.0861, 0.7854);
  addPose(path, 8.8306, 9.0806, 0.7854);
  addPose(path, 8.825, 9.075, 0.7854);
  addPose(path, 8.8194, 9.0694, 0.7854);
  addPose(path, 8.8139, 9.0639, 0.7854);
  addPose(path, 8.8083, 9.0583, 0.7854);
  addPose(path, 8.8028, 9.0528, 0.7854);
  addPose(path, 8.7972, 9.0472, 0.7854);
  addPose(path, 8.7917, 9.0417, 0.7854);
  addPose(path, 8.7861, 9.0361, 0.7854);
  addPose(path, 8.7806, 9.0306, 0.7854);
  addPose(path, 8.775, 9.025, 0.7854);
  addPose(path, 8.7694, 9.0194, 0.7854);
  addPose(path, 8.7639, 9.0139, 0.7854);
  addPose(path, 8.7583, 9.0083, 0.7854);
  addPose(path, 8.7528, 9.0028, 0.7854);
  addPose(path, 8.7472, 8.9972, 0.7854);
  addPose(path, 8.7417, 8.9917, 0.7854);
  addPose(path, 8.7361, 8.9861, 0.7854);
  addPose(path, 8.7306, 8.9806, 0.7854);
  addPose(path, 8.725, 8.975, 0.7854);
  addPose(path, 8.7194, 8.9694, 0.7854);
  addPose(path, 8.7139, 8.9639, 0.7854);
  addPose(path, 8.7083, 8.9583, 0.7854);
  addPose(path, 8.7028, 8.9528, 0.7854);
  addPose(path, 8.6972, 8.9472, 0.7854);
  addPose(path, 8.6917, 8.9417, 0.7854);
  addPose(path, 8.6861, 8.9361, 0.7854);
  addPose(path, 8.6806, 8.9306, 0.7854);
  addPose(path, 8.675, 8.925, 0.7854);
  addPose(path, 8.6694, 8.9194, 0.7854);
  addPose(path, 8.6639, 8.9139, 0.7854);
  addPose(path, 8.6583, 8.9083, 0.7854);
  addPose(path, 8.6528, 8.9028, 0.7854);
  addPose(path, 8.6472, 8.8972, 0.7854);
  addPose(path, 8.6417, 8.8917, 0.7854);
  addPose(path, 8.6361, 8.8861, 0.7854);
  addPose(path, 8.6306, 8.8806, 0.7854);
  addPose(path, 8.625, 8.875, 0.7854);
  addPose(path, 8.6194, 8.8694, 0.7854);
  addPose(path, 8.6139, 8.8639, 0.7854);
  addPose(path, 8.6083, 8.8583, 0.7854);
  addPose(path, 8.6028, 8.8528, 0.7854);
  addPose(path, 8.5972, 8.8472, 0.7854);
  addPose(path, 8.5917, 8.8417, 0.7854);
  addPose(path, 8.5861, 8.8361, 0.7854);
  addPose(path, 8.5806, 8.8306, 0.7854);
  addPose(path, 8.575, 8.825, 0.7854);
  addPose(path, 8.5694, 8.8194, 0.7854);
  addPose(path, 8.5639, 8.8139, 0.7854);
  addPose(path, 8.5583, 8.8083, 0.7854);
  addPose(path, 8.5528, 8.8028, 0.7854);
  addPose(path, 8.5472, 8.7972, 0.7854);
  addPose(path, 8.5417, 8.7917, 0.7854);
  addPose(path, 8.5361, 8.7861, 0.7854);
  addPose(path, 8.5306, 8.7806, 0.7854);
  addPose(path, 8.525, 8.775, 0.7854);
  addPose(path, 8.5194, 8.7694, 0.7854);
  addPose(path, 8.5139, 8.7639, 0.7854);
  addPose(path, 8.5083, 8.7583, 0.7854);
  addPose(path, 8.5028, 8.7528, 0.7854);
  addPose(path, 8.4972, 8.7472, 0.7854);
  addPose(path, 8.4917, 8.7417, 0.7854);
  addPose(path, 8.4861, 8.7361, 0.7854);
  addPose(path, 8.4806, 8.7306, 0.7854);
  addPose(path, 8.475, 8.725, 0.7854);
  addPose(path, 8.4694, 8.7194, 0.7854);
  addPose(path, 8.4639, 8.7139, 0.7854);
  addPose(path, 8.4583, 8.7083, 0.7854);
  addPose(path, 8.4528, 8.7028, 0.7854);
  addPose(path, 8.4472, 8.6972, 0.7854);
  addPose(path, 8.4417, 8.6917, 0.7854);
  addPose(path, 8.4361, 8.6861, 0.7854);
  addPose(path, 8.4306, 8.6806, 0.7854);
  addPose(path, 8.425, 8.675, 0.7854);
  addPose(path, 8.4194, 8.6694, 0.7854);
  addPose(path, 8.4139, 8.6639, 0.7854);
  addPose(path, 8.4083, 8.6583, 0.7854);
  addPose(path, 8.4028, 8.6528, 0.7854);
  addPose(path, 8.3972, 8.6472, 0.7854);
  addPose(path, 8.3917, 8.6417, 0.7854);
  addPose(path, 8.3861, 8.6361, 0.7854);
  addPose(path, 8.3806, 8.6306, 0.7854);
  addPose(path, 8.375, 8.625, 0.7854);
  addPose(path, 8.3694, 8.6194, 0.7854);
  addPose(path, 8.3639, 8.6139, 0.7854);
  addPose(path, 8.3583, 8.6083, 0.7854);
  addPose(path, 8.3528, 8.6028, 0.7854);
  addPose(path, 8.3472, 8.5972, 0.7854);
  addPose(path, 8.3417, 8.5917, 0.7854);
  addPose(path, 8.3361, 8.5861, 0.7854);
  addPose(path, 8.3306, 8.5806, 0.7854);
  addPose(path, 8.325, 8.575, 0.7854);
  addPose(path, 8.3194, 8.5694, 0.7854);
  addPose(path, 8.3139, 8.5639, 0.7854);
  addPose(path, 8.3083, 8.5583, 0.7854);
  addPose(path, 8.3028, 8.5528, 0.7854);
  addPose(path, 8.2972, 8.5472, 0.7854);
  addPose(path, 8.2917, 8.5417, 0.7854);
  addPose(path, 8.2861, 8.5361, 0.7854);
  addPose(path, 8.2806, 8.5306, 0.7854);
  addPose(path, 8.275, 8.525, 0.7854);
  addPose(path, 8.2694, 8.5194, 0.7854);
  addPose(path, 8.2639, 8.5139, 0.7854);
  addPose(path, 8.2583, 8.5083, 0.7854);
  addPose(path, 8.2528, 8.5028, 0.7854);
  addPose(path, 8.2472, 8.4972, 0.7854);
  addPose(path, 8.2417, 8.4917, 0.7854);
  addPose(path, 8.2361, 8.4861, 0.7854);
  addPose(path, 8.2306, 8.4806, 0.7854);
  addPose(path, 8.225, 8.475, 0.7854);
  addPose(path, 8.2194, 8.4694, 0.7854);
  addPose(path, 8.2139, 8.4639, 0.7854);
  addPose(path, 8.2083, 8.4583, 0.7854);
  addPose(path, 8.2028, 8.4528, 0.7854);
  addPose(path, 8.1972, 8.4472, 0.7854);
  addPose(path, 8.1917, 8.4417, 0.7854);
  addPose(path, 8.1861, 8.4361, 0.7854);
  addPose(path, 8.1806, 8.4306, 0.7854);
  addPose(path, 8.175, 8.425, 0.7854);
  addPose(path, 8.1694, 8.4194, 0.7854);
  addPose(path, 8.1639, 8.4139, 0.7854);
  addPose(path, 8.1583, 8.4083, 0.7854);
  addPose(path, 8.1528, 8.4028, 0.7854);
  addPose(path, 8.1472, 8.3972, 0.7854);
  addPose(path, 8.1417, 8.3917, 0.7854);
  addPose(path, 8.1361, 8.3861, 0.7854);
  addPose(path, 8.1306, 8.3806, 0.7854);
  addPose(path, 8.125, 8.375, 0.7854);
  addPose(path, 8.1194, 8.3694, 0.7854);
  addPose(path, 8.1139, 8.3639, 0.7854);
  addPose(path, 8.1083, 8.3583, 0.7854);
  addPose(path, 8.1028, 8.3528, 0.7854);
  addPose(path, 8.0972, 8.3472, 0.7854);
  addPose(path, 8.0917, 8.3417, 0.7854);
  addPose(path, 8.0861, 8.3361, 0.7854);
  addPose(path, 8.0806, 8.3306, 0.7854);
  addPose(path, 8.075, 8.325, 0.7854);
  addPose(path, 8.0694, 8.3194, 0.7854);
  addPose(path, 8.0639, 8.3139, 0.7854);
  addPose(path, 8.0583, 8.3083, 0.7854);
  addPose(path, 8.0528, 8.3028, 0.7854);
  addPose(path, 8.0472, 8.2972, 0.7854);
  addPose(path, 8.0417, 8.2917, 0.7854);
  addPose(path, 8.0361, 8.2861, 0.7854);
  addPose(path, 8.0306, 8.2806, 0.7854);
  addPose(path, 8.025, 8.275, 0.7854);
  addPose(path, 8.0194, 8.2694, 0.7854);
  addPose(path, 8.0139, 8.2639, 0.7854);
  addPose(path, 8.0083, 8.2583, 0.7854);
  addPose(path, 8.0028, 8.2528, 0.7854);
  addPose(path, 7.9972, 8.2472, 0.7854);
  addPose(path, 7.9917, 8.2417, 0.7854);
  addPose(path, 7.9861, 8.2361, 0.7854);
  addPose(path, 7.9806, 8.2306, 0.7854);
  addPose(path, 7.975, 8.225, 0.7854);
  addPose(path, 7.9694, 8.2194, 0.7854);
  addPose(path, 7.9639, 8.2139, 0.7854);
  addPose(path, 7.9583, 8.2083, 0.7854);
  addPose(path, 7.9528, 8.2028, 0.7854);
  addPose(path, 7.9472, 8.1972, 0.7854);
  addPose(path, 7.9417, 8.1917, 0.7854);
  addPose(path, 7.9361, 8.1861, 0.7854);
  addPose(path, 7.9306, 8.1806, 0.7854);
  addPose(path, 7.925, 8.175, 0.7854);
  addPose(path, 7.9194, 8.1694, 0.7854);
  addPose(path, 7.9139, 8.1639, 0.7854);
  addPose(path, 7.9083, 8.1583, 0.7854);
  addPose(path, 7.9028, 8.1528, 0.7854);
  addPose(path, 7.8972, 8.1472, 0.7854);
  addPose(path, 7.8917, 8.1417, 0.7854);
  addPose(path, 7.8861, 8.1361, 0.7854);
  addPose(path, 7.8806, 8.1306, 0.7854);
  addPose(path, 7.875, 8.125, 0.7854);
  addPose(path, 7.8694, 8.1194, 0.7854);
  addPose(path, 7.8639, 8.1139, 0.7854);
  addPose(path, 7.8583, 8.1083, 0.7854);
  addPose(path, 7.8528, 8.1028, 0.7854);
  addPose(path, 7.8472, 8.0972, 0.7854);
  addPose(path, 7.8417, 8.0917, 0.7854);
  addPose(path, 7.8361, 8.0861, 0.7854);
  addPose(path, 7.8306, 8.0806, 0.7854);
  addPose(path, 7.825, 8.075, 0.7854);
  addPose(path, 7.8194, 8.0694, 0.7854);
  addPose(path, 7.8139, 8.0639, 0.7854);
  addPose(path, 7.8083, 8.0583, 0.7854);
  addPose(path, 7.8028, 8.0528, 0.7854);
  addPose(path, 7.7972, 8.0472, 0.7854);
  addPose(path, 7.7917, 8.0417, 0.7854);
  addPose(path, 7.7861, 8.0361, 0.7854);
  addPose(path, 7.7806, 8.0306, 0.7854);
  addPose(path, 7.775, 8.025, 0.7854);
  addPose(path, 7.7694, 8.0194, 0.7854);
  addPose(path, 7.7639, 8.0139, 0.7854);
  addPose(path, 7.7583, 8.0083, 0.7854);
  addPose(path, 7.7528, 8.0028, 0.7854);
  addPose(path, 7.7472, 7.9972, 0.7854);
  addPose(path, 7.7417, 7.9917, 0.7854);
  addPose(path, 7.7361, 7.9861, 0.7854);
  addPose(path, 7.7306, 7.9806, 0.7854);
  addPose(path, 7.725, 7.975, 0.7854);
  addPose(path, 7.7194, 7.9694, 0.7854);
  addPose(path, 7.7139, 7.9639, 0.7854);
  addPose(path, 7.7083, 7.9583, 0.7854);
  addPose(path, 7.7028, 7.9528, 0.7854);
  addPose(path, 7.6972, 7.9472, 0.7854);
  addPose(path, 7.6917, 7.9417, 0.7854);
  addPose(path, 7.6861, 7.9361, 0.7854);
  addPose(path, 7.6806, 7.9306, 0.7854);
  addPose(path, 7.675, 7.925, 0.7854);
  addPose(path, 7.6694, 7.9194, 0.7854);
  addPose(path, 7.6639, 7.9139, 0.7854);
  addPose(path, 7.6583, 7.9083, 0.7854);
  addPose(path, 7.6528, 7.9028, 0.7854);
  addPose(path, 7.6472, 7.8972, 0.7854);
  addPose(path, 7.6417, 7.8917, 0.7854);
  addPose(path, 7.6361, 7.8861, 0.7854);
  addPose(path, 7.6306, 7.8806, 0.7854);
  addPose(path, 7.625, 7.875, 0.7854);
  addPose(path, 7.6194, 7.8694, 0.7854);
  addPose(path, 7.6139, 7.8639, 0.7854);
  addPose(path, 7.6083, 7.8583, 0.7854);
  addPose(path, 7.6028, 7.8528, 0.7854);
  addPose(path, 7.5972, 7.8472, 0.7854);
  addPose(path, 7.5917, 7.8417, 0.7854);
  addPose(path, 7.5861, 7.8361, 0.7854);
  addPose(path, 7.5806, 7.8306, 0.7854);
  addPose(path, 7.575, 7.825, 0.7854);
  addPose(path, 7.5694, 7.8194, 0.7854);
  addPose(path, 7.5639, 7.8139, 0.7854);
  addPose(path, 7.5583, 7.8083, 0.7854);
  addPose(path, 7.5528, 7.8028, 0.7854);
  addPose(path, 7.5472, 7.7972, 0.7854);
  addPose(path, 7.5417, 7.7917, 0.7854);
  addPose(path, 7.5361, 7.7861, 0.7854);
  addPose(path, 7.5306, 7.7806, 0.7854);
  addPose(path, 7.525, 7.775, 0.7854);

  // ======= segment 1 =========
  addPose(path, 7.5591, 7.8091, 0.7854);
  addPose(path, 7.5934, 7.8428, 0.7557);
  addPose(path, 7.6293, 7.875, 0.7039);
  addPose(path, 7.6668, 7.9052, 0.652);
  addPose(path, 7.7059, 7.9334, 0.6001);
  addPose(path, 7.7463, 7.9596, 0.5483);
  addPose(path, 7.7881, 7.9836, 0.4964);
  addPose(path, 7.831, 8.0054, 0.4446);
  addPose(path, 7.875, 8.025, 0.3927);

  // ======= segment 2 =========
  addPose(path, 7.8639, 8.0194, 0.3927);
  addPose(path, 7.8528, 8.0139, 0.3927);
  addPose(path, 7.8417, 8.0083, 0.3927);
  addPose(path, 7.8306, 8.0028, 0.3927);
  addPose(path, 7.8194, 7.9972, 0.3927);
  addPose(path, 7.8083, 7.9917, 0.3927);
  addPose(path, 7.7972, 7.9861, 0.3927);
  addPose(path, 7.7861, 7.9806, 0.3927);
  addPose(path, 7.775, 7.975, 0.3927);

  // ======= segment 3 =========
  addPose(path, 7.8127, 7.9906, 0.3927);
  addPose(path, 7.8504, 8.0062, 0.3927);
  addPose(path, 7.8881, 8.0218, 0.3927);
  addPose(path, 7.9258, 8.0373, 0.3736);
  addPose(path, 7.9643, 8.0508, 0.2989);
  addPose(path, 8.0037, 8.0613, 0.2242);
  addPose(path, 8.0437, 8.0689, 0.1494);
  addPose(path, 8.0842, 8.0735, 0.0747);
  addPose(path, 8.125, 8.075, 0);
  addPose(path, 8.1702, 8.075, 0);
  addPose(path, 8.2154, 8.075, 0);
  addPose(path, 8.2605, 8.075, 0);
  addPose(path, 8.3057, 8.0742, -0.0488);
  addPose(path, 8.3507, 8.0705, -0.1176);
  addPose(path, 8.3953, 8.0636, -0.1864);
  addPose(path, 8.4394, 8.0537, -0.2551);
  addPose(path, 8.4827, 8.0408, -0.3239);
  addPose(path, 8.525, 8.025, -0.392685);
  addPose(path, 8.5361, 8.0194, -0.392685);
  addPose(path, 8.5472, 8.0139, -0.392685);
  addPose(path, 8.5583, 8.0083, -0.392685);
  addPose(path, 8.5694, 8.0028, -0.392685);
  addPose(path, 8.5806, 7.9972, -0.392685);
  addPose(path, 8.5917, 7.9917, -0.392685);
  addPose(path, 8.6028, 7.9861, -0.392685);
  addPose(path, 8.6139, 7.9806, -0.392685);
  addPose(path, 8.625, 7.975, -0.392685);
  addPose(path, 8.6361, 7.9694, -0.392685);
  addPose(path, 8.6472, 7.9639, -0.392685);
  addPose(path, 8.6583, 7.9583, -0.392685);
  addPose(path, 8.6694, 7.9528, -0.392685);
  addPose(path, 8.6806, 7.9472, -0.392685);
  addPose(path, 8.6917, 7.9417, -0.392685);
  addPose(path, 8.7028, 7.9361, -0.392685);
  addPose(path, 8.7139, 7.9306, -0.392685);
  addPose(path, 8.725, 7.925, -0.392685);
  addPose(path, 8.7361, 7.9194, -0.392685);
  addPose(path, 8.7472, 7.9139, -0.392685);
  addPose(path, 8.7583, 7.9083, -0.392685);
  addPose(path, 8.7694, 7.9028, -0.392685);
  addPose(path, 8.7806, 7.8972, -0.392685);
  addPose(path, 8.7917, 7.8917, -0.392685);
  addPose(path, 8.8028, 7.8861, -0.392685);
  addPose(path, 8.8139, 7.8806, -0.392685);
  addPose(path, 8.825, 7.875, -0.392685);
  addPose(path, 8.8361, 7.8694, -0.392685);
  addPose(path, 8.8472, 7.8639, -0.392685);
  addPose(path, 8.8583, 7.8583, -0.392685);
  addPose(path, 8.8694, 7.8528, -0.392685);
  addPose(path, 8.8806, 7.8472, -0.392685);
  addPose(path, 8.8917, 7.8417, -0.392685);
  addPose(path, 8.9028, 7.8361, -0.392685);
  addPose(path, 8.9139, 7.8306, -0.392685);
  addPose(path, 8.925, 7.825, -0.392685);
  addPose(path, 8.9361, 7.8194, -0.392685);
  addPose(path, 8.9472, 7.8139, -0.392685);
  addPose(path, 8.9583, 7.8083, -0.392685);
  addPose(path, 8.9694, 7.8028, -0.392685);
  addPose(path, 8.9806, 7.7972, -0.392685);
  addPose(path, 8.9917, 7.7917, -0.392685);
  addPose(path, 9.0028, 7.7861, -0.392685);
  addPose(path, 9.0139, 7.7806, -0.392685);
  addPose(path, 9.025, 7.775, -0.392685);
  addPose(path, 9.0361, 7.7694, -0.392685);
  addPose(path, 9.0472, 7.7639, -0.392685);
  addPose(path, 9.0583, 7.7583, -0.392685);
  addPose(path, 9.0694, 7.7528, -0.392685);
  addPose(path, 9.0806, 7.7472, -0.392685);
  addPose(path, 9.0917, 7.7417, -0.392685);
  addPose(path, 9.1028, 7.7361, -0.392685);
  addPose(path, 9.1139, 7.7306, -0.392685);
  addPose(path, 9.125, 7.725, -0.392685);
  addPose(path, 9.1627, 7.7094, -0.392685);
  addPose(path, 9.2004, 7.6938, -0.392685);
  addPose(path, 9.2381, 7.6782, -0.392685);
  addPose(path, 9.2758, 7.6627, -0.373585);
  addPose(path, 9.3143, 7.6492, -0.298885);
  addPose(path, 9.3537, 7.6387, -0.224185);
  addPose(path, 9.3937, 7.6311, -0.149485);
  addPose(path, 9.4342, 7.6265, -0.0746853);

  std::vector<nav_2d_msgs::Path2D> segments = splitPlan(path);
  EXPECT_EQ(4U, segments.size());
  EXPECT_EQ(262U, segments[0].poses.size());
  EXPECT_EQ(10U, segments[1].poses.size());
  EXPECT_EQ(10U, segments[2].poses.size());
  EXPECT_EQ(81U, segments[3].poses.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
