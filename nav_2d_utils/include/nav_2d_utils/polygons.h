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

#ifndef NAV_2D_UTILS_POLYGONS_H
#define NAV_2D_UTILS_POLYGONS_H

#include <ros/ros.h>
#include <nav_2d_msgs/Polygon2D.h>
#include <nav_2d_msgs/ComplexPolygon2D.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <string>

namespace nav_2d_utils
{

/**
 * @class PolygonParseException
 * @brief Exception to throw when Polygon doesn't load correctly
 */
class PolygonParseException: public std::runtime_error
{
public:
  explicit PolygonParseException(const std::string& description) : std::runtime_error(description) {}
};

/**
 * @brief Parse a vector of vectors of doubles from a string.
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...]
 *
 * @param input The string to parse
 * @return a vector of vectors of doubles
 */
std::vector<std::vector<double> > parseVVD(const std::string& input);

/**
 * @brief Create a "circular" polygon from a given radius
 *
 * @param radius Radius of the polygon
 * @param num_points Number of points in the resulting polygon
 * @return A rotationally symmetric polygon with the specified number of vertices
 */
nav_2d_msgs::Polygon2D polygonFromRadius(const double radius, const unsigned int num_points = 16);

/**
 * @brief Make a polygon from the given string.
 * Format should be bracketed array of arrays of doubles, like so: [[1.0, 2.2], [3.3, 4.2], ...]
 *
 * @param polygon_string The string to parse
 * @return Polygon2D
 */
nav_2d_msgs::Polygon2D polygonFromString(const std::string& polygon_string);

/**
 * @brief Load a polygon from a parameter, whether it is a string or array, or two arrays
 * @param nh Node handle to load parameter from
 * @param parameter_name Name of the parameter
 * @param search Whether to search up the namespace for the parameter name
 * @return Loaded polygon
 */
nav_2d_msgs::Polygon2D polygonFromParams(const ros::NodeHandle& nh, const std::string parameter_name,
                                         bool search = true);

/**
 * @brief Create a polygon from the given XmlRpcValue.
 *
 * @param polygon_xmlrpc should be an array of arrays, where the top-level array should have
 *                       3 or more elements, and the sub-arrays should all have exactly 2 elements
 *                       (x and y coordinates).
 */
nav_2d_msgs::Polygon2D polygonFromXMLRPC(XmlRpc::XmlRpcValue& polygon_xmlrpc);

/**
 * @brief Create XMLRPC Value for writing the polygon to the parameter server
 * @param polygon Polygon to convert
 * @param array_of_arrays If true, write an array of arrays. Otherwise, write two parallel arrays
 * @return XmlRpcValue
 */
XmlRpc::XmlRpcValue polygonToXMLRPC(const nav_2d_msgs::Polygon2D& polygon, bool array_of_arrays = true);

/**
 * @brief Save a polygon to a parameter
 * @param polygon The Polygon
 * @param nh Node handle to save the parameter to
 * @param parameter_name Name of the parameter
 * @param array_of_arrays If true, write an array of arrays. Otherwise, write two parallel arrays
 */
void polygonToParams(const nav_2d_msgs::Polygon2D& polygon, const ros::NodeHandle& nh, const std::string parameter_name,
                     bool array_of_arrays = true);

/**
 * @brief Create a polygon from two parallel arrays
 *
 * @param xs Array of doubles representing x coordinates, at least three elements long
 * @param ys Array of doubles representing y coordinates, matching length of xs
 */
nav_2d_msgs::Polygon2D polygonFromParallelArrays(const std::vector<double>& xs, const std::vector<double>& ys);

/**
 * @brief Create two parallel arrays from a polygon
 *
 * @param[in] polygon
 * @param[out] xs Array of doubles representing x coordinates, to be populated
 * @param[out] ys Array of doubles representing y coordinates, to be populated
 */
void polygonToParallelArrays(const nav_2d_msgs::Polygon2D polygon, std::vector<double>& xs, std::vector<double>& ys);

/**
 * @brief check if two polygons are equal. Used in testing
 */
bool equals(const nav_2d_msgs::Polygon2D& polygon0, const nav_2d_msgs::Polygon2D& polygon1);

/**
 * @brief Translate and rotate a polygon to a new pose
 * @param polygon The polygon
 * @param pose The x, y and theta to use when moving the polygon
 * @return A new moved polygon
 */
nav_2d_msgs::Polygon2D movePolygonToPose(const nav_2d_msgs::Polygon2D& polygon,
                                         const geometry_msgs::Pose2D& pose);

/**
 * @brief Check if a given point is inside of a polygon
 *
 * Borders are considered outside.
 *
 * @param polygon Polygon to check
 * @param x x coordinate
 * @param y y coordinate
 * @return true if point is inside polygon
 */
bool isInside(const nav_2d_msgs::Polygon2D& polygon, const double x, const double y);

/**
 * @brief Calculate the minimum and maximum distance from (0, 0) to any point on the polygon
 * @param[in] polygon polygon to analyze
 * @param[out] min_dist
 * @param[out] max_dist
 */
void calculateMinAndMaxDistances(const nav_2d_msgs::Polygon2D& polygon, double& min_dist, double& max_dist);

/**
 * @brief Decompose a complex polygon into a set of triangles.
 *
 * See https://en.wikipedia.org/wiki/Polygon_triangulation
 *
 * Implementation from https://github.com/mapbox/earcut.hpp
 *
 * @param polygon The complex polygon to deconstruct
 * @return A vector of points where each set of three points represents a triangle
 */
std::vector<nav_2d_msgs::Point2D> triangulate(const nav_2d_msgs::ComplexPolygon2D& polygon);

/**
 * @brief Decompose a simple polygon into a set of triangles.
 *
 * See https://en.wikipedia.org/wiki/Polygon_triangulation
 *
 * Implementation from https://github.com/mapbox/earcut.hpp
 *
 * @param polygon The polygon to deconstruct
 * @return A vector of points where each set of three points represents a triangle
 */
std::vector<nav_2d_msgs::Point2D> triangulate(const nav_2d_msgs::Polygon2D& polygon);

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS_POLYGONS_H
