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
#include <map_server/image_loader.h>
#include <nav_grid_server/image_loader.h>
#include <ros/package.h>
#include <string>
#include <vector>

std::vector<std::string> filenames = {"spectrum.png", "spectrum.pgm"};

bool compareToClassic(const std::string& filename, bool should_negate, double occ_th, double free_th, MapMode mode)
{
  const double resolution = 0.1;
  double origin[3] = {0.0, 0.0, 0.0};
  nav_msgs::GetMap::Response map_resp;

  std::string path = ros::package::getPath("nav_grid_server") + "/test/data/" + filename;

  map_server::loadMapFromFile(&map_resp, path.c_str(), resolution, should_negate, occ_th, free_th, origin, mode);

  std::string mode_str;
  if (mode == RAW)
  {
    mode_str = "raw";
  }
  else if (mode == TRINARY)
  {
    mode_str = "trinary";
  }
  else
  {
    mode_str = "scale";
  }

  nav_grid::VectorNavGrid<unsigned char> the_map =
    nav_grid_server::classicLoadMapFromFile(path, resolution, should_negate, occ_th, free_th, mode_str);

  if (the_map.size() != map_resp.map.data.size())
  {
    return false;
  }
  unsigned int correct = 0;
  for (unsigned int i=0; i < the_map.size(); i++)
  {
    unsigned char original = map_resp.map.data[i];
    unsigned char updated = the_map[i];
    if (original == updated)
      correct += 1;
  }
  if (correct != the_map.size())
  {
    printf("%.2f correct\n", correct * 100.0 / the_map.size());
    for (unsigned int i=0; i < the_map.size(); i++)
    {
      unsigned char original = map_resp.map.data[i];
      unsigned char updated = the_map[i];
      if (original == updated)
        correct += 1;
      printf("%d) %d %d %c\n", i, updated, original, updated == original ? ' ' : 'x');
    }
  }

  return correct == the_map.size();
}

TEST(MapServerComparison, raw)
{
  for (auto& filename : filenames)
  {
    EXPECT_TRUE(compareToClassic(filename, false, 0.9, 0.1, RAW)) << filename;
    EXPECT_TRUE(compareToClassic(filename, true, 0.9, 0.1, RAW)) << filename;
  }
}

TEST(MapServerComparison, trinary)
{
  for (auto& filename : filenames)
  {
    EXPECT_TRUE(compareToClassic(filename, true, 0.65, 0.196, TRINARY)) << filename;
    EXPECT_TRUE(compareToClassic(filename, true, 0.85, 0.15, TRINARY)) << filename;
    EXPECT_TRUE(compareToClassic(filename, false, 0.65, 0.196, TRINARY)) << filename;
    EXPECT_TRUE(compareToClassic(filename, false, 0.85, 0.15, TRINARY)) << filename;
  }
}

TEST(MapServerComparison, scale)
{
  for (auto& filename : filenames)
  {
    EXPECT_TRUE(compareToClassic(filename, true, 0.65, 0.196, SCALE)) << filename;
    EXPECT_TRUE(compareToClassic(filename, true, 0.85, 0.15, SCALE)) << filename;
    EXPECT_TRUE(compareToClassic(filename, false, 0.65, 0.196, SCALE)) << filename;
    EXPECT_TRUE(compareToClassic(filename, false, 0.85, 0.15, SCALE)) << filename;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
