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

#ifndef NAV_GRID_PUB_SUB_COST_INTERPRETATION_TABLES_H
#define NAV_GRID_PUB_SUB_COST_INTERPRETATION_TABLES_H

#include <vector>

namespace nav_grid_pub_sub
{
/**
 * @brief Every value is untouched
 */
static std::vector<unsigned char> RAW(0);

/**
 * @brief Every value is 255 - i value
 *
 * map from [0, 255] to [255, 0]
 */
static std::vector<unsigned char> NEGATE =
{
  // [0, 255] --> 255 - i
  255, 254, 253, 252, 251, 250, 249, 248, 247, 246,   // [0, 9]
  245, 244, 243, 242, 241, 240, 239, 238, 237, 236,
  235, 234, 233, 232, 231, 230, 229, 228, 227, 226,
  225, 224, 223, 222, 221, 220, 219, 218, 217, 216,
  215, 214, 213, 212, 211, 210, 209, 208, 207, 206,
  205, 204, 203, 202, 201, 200, 199, 198, 197, 196,   // [50, 59]
  195, 194, 193, 192, 191, 190, 189, 188, 187, 186,
  185, 184, 183, 182, 181, 180, 179, 178, 177, 176,
  175, 174, 173, 172, 171, 170, 169, 168, 167, 166,
  165, 164, 163, 162, 161, 160, 159, 158, 157, 156,
  155, 154, 153, 152, 151, 150, 149, 148, 147, 146,   // [100, 109]
  145, 144, 143, 142, 141, 140, 139, 138, 137, 136,
  135, 134, 133, 132, 131, 130, 129, 128, 127, 126,
  125, 124, 123, 122, 121, 120, 119, 118, 117, 116,
  115, 114, 113, 112, 111, 110, 109, 108, 107, 106,
  105, 104, 103, 102, 101, 100, 99, 98, 97, 96,       // [150, 159]
  95, 94, 93, 92, 91, 90, 89, 88, 87, 86,
  85, 84, 83, 82, 81, 80, 79, 78, 77, 76,
  75, 74, 73, 72, 71, 70, 69, 68, 67, 66,
  65, 64, 63, 62, 61, 60, 59, 58, 57, 56,
  55, 54, 53, 52, 51, 50, 49, 48, 47, 46,             // [200, 209]
  45, 44, 43, 42, 41, 40, 39, 38, 37, 36,
  35, 34, 33, 32, 31, 30, 29, 28, 27, 26,
  25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
  15, 14, 13, 12, 11, 10, 9, 8, 7, 6,
  5, 4, 3, 2, 1, 0                                    // [250, 255]
};

/**
 * @brief Above 100 is occupied, -1 is sometimes unknown, and otherwise it is either scaled or freespace.
 *
 * Expects values from [-1, 100] and outputs [0, 255]
 */
std::vector<unsigned char> getOccupancyInput(bool trinary = false, bool use_unknown_value = false);

/**
 * @brief Scale [0, 255] down to [0, 100] (except for a few special values)
 *
 * Expects values from [0, 255] and flattens them to [-1, 100]
 * There are a few special values that only have one input value mapping to them.
 * 0, 253, 254 and 255 always map to 0, 99, 100, and 255 (and are the only values that do).
 */
static std::vector<unsigned char> OCC_GRID_PUBLISHING =
{
  // 0: NO obstacle
  0,
  // [1, 252]: scale to [1, 98] i.e. static_cast<char>(1 + (97 * (i - 1)) / 251)
  1, 1, 1, 2, 2, 2, 3, 3, 4, 4,            // [1, 10]
  4, 5, 5, 6, 6, 6, 7, 7, 7, 8,            // [11, 20]
  8, 9, 9, 9, 10, 10, 11, 11, 11, 12,
  12, 12, 13, 13, 14, 14, 14, 15, 15, 16,
  16, 16, 17, 17, 18, 18, 18, 19, 19, 19,  // [41, 50]
  20, 20, 21, 21, 21, 22, 22, 23, 23, 23,
  24, 24, 24, 25, 25, 26, 26, 26, 27, 27,
  28, 28, 28, 29, 29, 29, 30, 30, 31, 31,
  31, 32, 32, 33, 33, 33, 34, 34, 35, 35,
  35, 36, 36, 36, 37, 37, 38, 38, 38, 39,  // [91, 100]
  39, 40, 40, 40, 41, 41, 41, 42, 42, 43,
  43, 43, 44, 44, 45, 45, 45, 46, 46, 46,
  47, 47, 48, 48, 48, 49, 49, 50, 50, 50,
  51, 51, 52, 52, 52, 53, 53, 53, 54, 54,
  55, 55, 55, 56, 56, 57, 57, 57, 58, 58,  // [141, 150]
  58, 59, 59, 60, 60, 60, 61, 61, 62, 62,
  62, 63, 63, 63, 64, 64, 65, 65, 65, 66,
  66, 67, 67, 67, 68, 68, 69, 69, 69, 70,
  70, 70, 71, 71, 72, 72, 72, 73, 73, 74,
  74, 74, 75, 75, 75, 76, 76, 77, 77, 77,  // [191, 200]
  78, 78, 79, 79, 79, 80, 80, 80, 81, 81,
  82, 82, 82, 83, 83, 84, 84, 84, 85, 85,
  86, 86, 86, 87, 87, 87, 88, 88, 89, 89,
  89, 90, 90, 91, 91, 91, 92, 92, 92, 93,
  93, 94, 94, 94, 95, 95, 96, 96, 96, 97,  // [241, 250]
  97, 98,                                  // [251, 252]
  // 253: inscribed obstacle
  99,
  // 254: lethal obstacle
  100,
  // 255: unknown
  255
};

/**
 * @brief Above occupied_threshold is occupied, below free_threshold is free, and the middle gray is unknown
 *
 * Expects values from [0, 255] and flattens them to 0, 100, and -1
 */
std::vector<unsigned char> pixelColoringInterpretation(const double free_threshold,
                                                       const double occupied_threshold);

/**
 * @brief Above occupied_threshold is occupied, below free_threshold is free, and the middle is in between
 *
 * Expects values from [0, 255] and flattens them to [0, 100]
 */
std::vector<unsigned char> grayScaleInterpretation(const double free_threshold, const double occupied_threshold);

/**
 * @brief 0 becomes 254 (white), 100 becomes 0 (black) and everything else is 205 (gray)
 *
 * Translates from a trinary occupancy grid to an image pixel color
 */
static std::vector<unsigned char> TRINARY_SAVE =
{
  // 0: NO obstacle --> white
  255,
  // [1, 253] --> gray
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [1, 10]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [11, 20]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [41, 50]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [91, 100]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [101, 110]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [141, 150]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [191, 200]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [241, 250]
  205, 205, 205,                                     // [251, 253]
  // 254: --> black
  0,
  // 255 --> gray
  205
};

/**
 * @brief 0 becomes 254 (white), 100 becomes 0 (black) and everything in between is gray
 *
 * Translates from a trinary occupancy grid to an image pixel color
 */
static std::vector<unsigned char> SCALE_SAVE =
{
  // 0: NO obstacle --> white
  254,
  // [1, 99] --> scale from white to black
  252, 250, 247, 245, 242, 240, 237, 234, 232, 229,  // [1, 10]
  227, 224, 221, 219, 216, 214, 211, 209, 206, 203,
  201, 198, 196, 193, 190, 188, 185, 183, 180, 178,
  175, 172, 170, 167, 165, 162, 159, 157, 154, 152,
  149, 147, 144, 141, 139, 136, 134, 131, 129, 126,  // [41, 50]
  123, 121, 118, 116, 113, 110, 108, 105, 103, 100,
  98, 95, 92, 90, 87, 85, 82, 79, 77, 74,
  72, 69, 67, 64, 61, 59, 56, 54, 51, 48,
  46, 43, 41, 38, 36, 33, 30, 28, 25, 23,
  20, 17, 15, 12, 10, 7, 5, 2, 1,                    // [91, 99]
  // 100 --> black
  0,
  // [101, 255] --> grey
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [101, 110]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [141, 150]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [191, 200]
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,
  205, 205, 205, 205, 205, 205, 205, 205, 205, 205,  // [241, 250]
  205, 205, 205, 205, 205                            // [251, 255]
};

}  // namespace nav_grid_pub_sub

#endif  // NAV_GRID_PUB_SUB_COST_INTERPRETATION_TABLES_H
