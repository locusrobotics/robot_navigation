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

#include <nav_grid_pub_sub/cost_interpretation_tables.h>
#include <vector>

namespace nav_grid_pub_sub
{

std::vector<unsigned char> pixelColoringInterpretation(const double free_threshold, const double occupied_threshold)
{
  std::vector<unsigned char> cost_interpretation_table(256);
  for (unsigned int i = 0; i < cost_interpretation_table.size(); i++)
  {
    double intensity = static_cast<double>(i) / 255.0;
    if (intensity > occupied_threshold)
    {
      cost_interpretation_table[i] = +100;
    }
    else if (intensity < free_threshold)
    {
      cost_interpretation_table[i] = 0;
    }
    else
    {
      cost_interpretation_table[i] = -1;
    }
  }
  return cost_interpretation_table;
}

std::vector<unsigned char> grayScaleInterpretation(const double free_threshold, const double occupied_threshold)
{
  std::vector<unsigned char> cost_interpretation_table(256);
  for (unsigned int i = 0; i < cost_interpretation_table.size(); i++)
  {
    double intensity = static_cast<double>(i) / 255.0;
    if (intensity > occupied_threshold)
    {
      cost_interpretation_table[i] = +100;
    }
    else if (intensity < free_threshold)
    {
      cost_interpretation_table[i] = 0;
    }
    else
    {
      // scale from [free_threshold, occupied_threshold] to [1, 99]
      cost_interpretation_table[i] = 1 + 98 * (intensity - free_threshold) / (occupied_threshold - free_threshold);
    }
  }
  return cost_interpretation_table;
}

std::vector<unsigned char> getOccupancyInput(bool trinary, bool use_unknown_value)
{
  std::vector<unsigned char> cost_interpretation_table(256);
  for (unsigned int i = 0; i < cost_interpretation_table.size(); i++)
  {
    unsigned char value;
    if (use_unknown_value && i == 255)
      value = 255;
    else if (!use_unknown_value && i == 255)
      value = 0;
    else if (i >= 100)
      value = 254;
    else if (i == 99)
      value = 253;
    else if (trinary)
      value = 0;
    else
    {
      double scale = static_cast<double>(i) / 100.0;
      value = static_cast<unsigned char>(scale * 254);
    }
    cost_interpretation_table[i] = value;
  }
  return cost_interpretation_table;
}

}  // namespace nav_grid_pub_sub
