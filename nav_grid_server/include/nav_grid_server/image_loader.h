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

#ifndef NAV_GRID_SERVER_IMAGE_LOADER_H
#define NAV_GRID_SERVER_IMAGE_LOADER_H

#include <nav_grid/vector_nav_grid.h>
#include <vector>
#include <string>

namespace nav_grid_server
{
/**
 * @brief Independent (i.e. not OpenCV) representation of a Pixel
 *
 * Each channel's value is represented as a value [0.0, 1.0]
 */
struct Pixel
{
  Pixel() = default;
  std::vector<double> channels;
};

/**
 * @brief Load an image as a NavGrid of pixels
 *
 * @param filepath Path to image file
 * @param flip_y_axis Flag for whether to flip the values on their y axis
 *
 * Images are typically defined with their top left corner being (0,0) with increasing x values to the right
 * and increasing y values as you go down in the image.
 *
 * However, the standard coordinate frame still has x values increasing to the right, but increasing
 * y values go "up" now. Hence, the default behavior is to flip the y axis so that the map grid looks
 * the same as the image when put onto the coordinate frame.
 */
nav_grid::VectorNavGrid<Pixel> getImage(const std::string& filepath, bool flip_y_axis = true);

/**
 * @brief Load an image from a file and return the image intensity at each pixel.
 *
 * Resulting values are [0.0, 1.0] plus -1.0 if the image is transparent.
 */
nav_grid::VectorNavGrid<double> getImageIntensity(const std::string& filepath, bool flip_y_axis = true);

/**
 * @brief Load an image from a file and return values [0, 255]
 *
 * Transparent pixels are saved as 255 (i.e. UNKNOWN). Other values are scaled [0, 255]
 */
nav_grid::VectorNavGrid<unsigned char> getCostmapFromImage(const std::string& filepath, bool flip_y_axis = true);

/**
 * @brief Load an image from a file, mimicking map_server's loading style
 * Resulting values are [0, 100] and -1
 *
 * @param filepath Path to image file
 * @param resolution resolution of resulting NavGrid
 * @param negate_param Whether to negate all the values
 * @param occ_th Threshold above which values are 100 (if mode!="raw")
 * @param free_th Threshold below which values are 0 (if mode!="raw")
 * @param mode If mode is "raw", no interpretation of values is done. If mode is "trinary",
 *             the thresholds will be used, and everything else is marked as -1.
 *             Otherwise, the thresholds are used and everything else is scaled from [1, 99].
 */
nav_grid::VectorNavGrid<unsigned char> classicLoadMapFromFile(const std::string& filepath, const double resolution,
    const bool negate_param, const double occ_th, const double free_th, const std::string& mode);

}  // namespace nav_grid_server

#endif  // NAV_GRID_SERVER_IMAGE_LOADER_H
