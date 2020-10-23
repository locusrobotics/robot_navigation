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

#include <nav_grid_server/image_loader.h>
#include <nav_grid_iterators/whole_grid.h>
#include <nav_grid_pub_sub/cost_interpretation.h>
#include <nav_grid_pub_sub/cost_interpretation_tables.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace nav_grid_server
{
template <typename T>
double scaleNumber(T raw_value)
{
  return static_cast<double>(raw_value) / std::numeric_limits<T>::max();
}

template<typename T>
nav_grid::VectorNavGrid<Pixel> getImage(const cv::Mat& cv_img, bool flip_y_axis = false)
{
  nav_grid::VectorNavGrid<Pixel> image;
  nav_grid::NavGridInfo info;
  info.width = cv_img.cols;
  info.height = cv_img.rows;
  image.setInfo(info);

  unsigned int n_channels = cv_img.channels();

  for (unsigned int j = 0; j < info.height; j++)
  {
    const T* row_ptr = cv_img.ptr<T>(j);
    for (unsigned int i = 0; i < info.width; i++)
    {
      Pixel pixel;
      const T* col_ptr = row_ptr + i * n_channels;
      for (unsigned int k = 0; k < n_channels; ++k)
      {
        T raw_value = *(col_ptr + k);
        pixel.channels.push_back(scaleNumber(raw_value));
      }

      if (flip_y_axis)
        image.setValue(i, info.height - j - 1, pixel);
      else
        image.setValue(i, j, pixel);
    }
  }
  return image;
}

nav_grid::VectorNavGrid<Pixel> getImage(const std::string& filepath, bool flip_y_axis)
{
  cv::Mat cv_img = cv::imread(filepath, -1);  // -1 keeps the image in its native format, including alpha channel
  if (cv_img.data == nullptr)
  {
    std::string errmsg = std::string("failed to open image file \"") + filepath + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  int depth = cv_img.depth();
  switch (depth)
  {
    case CV_8U:  return getImage<unsigned char>(cv_img, flip_y_axis);
    case CV_8S:  return getImage<char>(cv_img, flip_y_axis);
    case CV_16U: return getImage<uint16_t>(cv_img, flip_y_axis);
    case CV_16S: return getImage<int16_t>(cv_img, flip_y_axis);
    case CV_32S: return getImage<int>(cv_img, flip_y_axis);
    case CV_32F: return getImage<float>(cv_img, flip_y_axis);
    case CV_64F: return getImage<double>(cv_img, flip_y_axis);
    default:
    {
      throw std::runtime_error("Unsupported image");
    }
  }
}

nav_grid::VectorNavGrid<double> getImageIntensity(const std::string& filepath, bool flip_y_axis)
{
  nav_grid::VectorNavGrid<Pixel> image = getImage(filepath, flip_y_axis);
  nav_grid::NavGridInfo info = image.getInfo();
  nav_grid::VectorNavGrid<double> intensity;
  intensity.setInfo(info);
  for (nav_grid::Index index : nav_grid_iterators::WholeGrid(info))
  {
    const Pixel& pixel = image(index);
    unsigned int max_channel = pixel.channels.size();
    if (max_channel == 4)
    {
      if (pixel.channels.back() == 0.0)
      {
        intensity.setValue(index, -1.0);
        continue;
      }
      max_channel--;
    }
    double sum = std::accumulate(pixel.channels.begin(), pixel.channels.begin() + max_channel, 0.0);
    intensity.setValue(index, sum / static_cast<double>(max_channel));
  }

  return intensity;
}

nav_grid::VectorNavGrid<unsigned char> getCostmapFromImage(const std::string& filepath, bool flip_y_axis)
{
  nav_grid::VectorNavGrid<double> intensity_img = getImageIntensity(filepath, flip_y_axis);
  nav_grid::NavGridInfo info = intensity_img.getInfo();
  nav_grid::VectorNavGrid<unsigned char> costmap;
  costmap.setInfo(info);
  for (nav_grid::Index index : nav_grid_iterators::WholeGrid(info))
  {
    double intensity = intensity_img(index);
    if (intensity < 0.0)
    {
      costmap.setValue(index, -1);
    }
    else
    {
      costmap.setValue(index, static_cast<unsigned char>(round(255 * intensity)));
    }
  }
  return costmap;
}

nav_grid::VectorNavGrid<unsigned char> classicLoadMapFromFile(const std::string& filepath, const double resolution,
    const bool negate_param, const double occ_th, const double free_th, const std::string& mode)
{
  nav_grid::VectorNavGrid<unsigned char> costmap = getCostmapFromImage(filepath);
  nav_grid::NavGridInfo full_info = costmap.getInfo();
  full_info.resolution = resolution;
  costmap.setInfo(full_info);

  if (mode == "raw")
  {
    // This line would have no effect, but you can run it if you really want to
    // nav_grid_pub_sub::applyInterpretation(costmap, nav_grid_pub_sub::RAW);

    if (negate_param)
    {
      nav_grid_pub_sub::applyInterpretation(costmap, nav_grid_pub_sub::NEGATE);
    }
  }
  else
  {
    if (!negate_param)
    {
      nav_grid_pub_sub::applyInterpretation(costmap, nav_grid_pub_sub::NEGATE);
    }

    if (mode == "trinary")
      nav_grid_pub_sub::applyInterpretation(costmap, nav_grid_pub_sub::pixelColoringInterpretation(free_th, occ_th));
    else
      nav_grid_pub_sub::applyInterpretation(costmap, nav_grid_pub_sub::grayScaleInterpretation(free_th, occ_th));
  }
  return costmap;
}

}  // namespace nav_grid_server
