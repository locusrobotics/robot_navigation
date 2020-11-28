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

#include <nav_2d_utils/polygons.h>
#include <nav_2d_utils/geometry_help.h>
#include <mapbox/earcut.hpp>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace nav_2d_utils
{

using nav_2d_msgs::Point2D;
using nav_2d_msgs::Polygon2D;

std::vector<std::vector<double> > parseVVD(const std::string& input)
{
  std::vector<std::vector<double> > result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<double> current_vector;
  while (input_ss.good())
  {
    switch (input_ss.peek())
    {
    case EOF:
      break;
    case '[':
      depth++;
      if (depth > 2)
      {
        throw PolygonParseException("Array depth greater than 2");
      }
      input_ss.get();
      current_vector.clear();
      break;
    case ']':
      depth--;
      if (depth < 0)
      {
        throw PolygonParseException("More close ] than open [");
      }
      input_ss.get();
      if (depth == 1)
      {
        result.push_back(current_vector);
      }
      break;
    case ',':
    case ' ':
    case '\t':
      input_ss.get();
      break;
    default:  // All other characters should be part of the numbers.
      if (depth != 2)
      {
        std::stringstream err_ss;
        err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
        throw PolygonParseException(err_ss.str());
      }
      double value;
      if (input_ss >> value)
      {
        current_vector.push_back(value);
      }
      break;
    }
  }

  if (depth != 0)
  {
    throw PolygonParseException("Unterminated vector string.");
  }

  return result;
}

Polygon2D polygonFromRadius(const double radius, const unsigned int num_points)
{
  Polygon2D polygon;
  Point2D pt;
  for (unsigned int i = 0; i < num_points; ++i)
  {
    double angle = i * 2 * M_PI / num_points;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;
    polygon.points.push_back(pt);
  }

  return polygon;
}

Polygon2D polygonFromString(const std::string& polygon_string)
{
  Polygon2D polygon;
  // Will throw PolygonParseException if problematic
  std::vector<std::vector<double> > vvd = parseVVD(polygon_string);

  // convert vvd into points.
  if (vvd.size() < 3)
  {
    throw PolygonParseException("You must specify at least three points for the polygon.");
  }

  polygon.points.resize(vvd.size());
  for (unsigned int i = 0; i < vvd.size(); i++)
  {
    if (vvd[ i ].size() != 2)
    {
      std::stringstream err_ss;
      err_ss << "Points in the polygon specification must be pairs of numbers. Point index " << i << " had ";
      err_ss << int(vvd[ i ].size()) << " numbers.";
      throw PolygonParseException(err_ss.str());
    }

    polygon.points[i].x = vvd[i][0];
    polygon.points[i].y = vvd[i][1];
  }

  return polygon;
}


/**
 * @brief Helper function. Convert value to double
 */
double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    return static_cast<double>(static_cast<int>(value));
  }
  else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    return static_cast<double>(value);
  }

  std::stringstream err_ss;
  err_ss << "Values in the polygon specification must be numbers. Found value: " << value.toXml();
  throw PolygonParseException(err_ss.str());
}

/**
 * @brief Helper function. Convert value to double array
 */
std::vector<double> getNumberVectorFromXMLRPC(XmlRpc::XmlRpcValue& value)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    throw PolygonParseException("Subarray must have type list.");
  }
  std::vector<double> array;
  for (int i = 0; i < value.size(); i++)
  {
    array.push_back(getNumberFromXMLRPC(value[i]));
  }
  return array;
}

Polygon2D polygonFromXMLRPC(XmlRpc::XmlRpcValue& polygon_xmlrpc)
{
  if (polygon_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
      polygon_xmlrpc != "" && polygon_xmlrpc != "[]")
  {
    return polygonFromString(std::string(polygon_xmlrpc));
  }

  if (polygon_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (!polygon_xmlrpc.hasMember("x") || !polygon_xmlrpc.hasMember("y"))
    {
      throw PolygonParseException("Dict-like Polygon must specify members x and y.");
    }
    std::vector<double> xs = getNumberVectorFromXMLRPC(polygon_xmlrpc["x"]);
    std::vector<double> ys = getNumberVectorFromXMLRPC(polygon_xmlrpc["y"]);
    return polygonFromParallelArrays(xs, ys);
  }

  // Make sure we have an array of at least 3 elements.
  if (polygon_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    std::stringstream err_ss;
    err_ss << "Polygon must be specified as a list of lists. Found object of type " << polygon_xmlrpc.getType()
           << " instead.";
    throw PolygonParseException(err_ss.str());
  }
  else if (polygon_xmlrpc.size() < 3)
  {
    throw PolygonParseException("You must specify at least three points for the polygon.");
  }

  Polygon2D polygon;
  Point2D pt;
  for (int i = 0; i < polygon_xmlrpc.size(); ++i)
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue& point_xml = polygon_xmlrpc[i];
    if (point_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      std::stringstream err_ss;
      err_ss << "Each point must be specified as a list. Found object of type " << point_xml.getType() << " instead.";
      throw PolygonParseException(err_ss.str());
    }
    else if (point_xml.size() != 2)
    {
      throw PolygonParseException("Each point must have two numbers (x and y).");
    }

    pt.x = getNumberFromXMLRPC(point_xml[0]);
    pt.y = getNumberFromXMLRPC(point_xml[1]);
    polygon.points.push_back(pt);
  }
  return polygon;
}

Polygon2D polygonFromParams(const ros::NodeHandle& nh, const std::string parameter_name, bool search)
{
  std::string full_param_name;
  if (search)
  {
    nh.searchParam(parameter_name, full_param_name);
  }
  else
  {
    full_param_name = parameter_name;
  }

  if (!nh.hasParam(full_param_name))
  {
    std::stringstream err_ss;
    err_ss << "Parameter " << parameter_name << "(" + nh.resolveName(parameter_name) << ") not found.";
    throw PolygonParseException(err_ss.str());
  }
  XmlRpc::XmlRpcValue polygon_xmlrpc;
  nh.getParam(full_param_name, polygon_xmlrpc);
  return polygonFromXMLRPC(polygon_xmlrpc);
}

/**
 * @brief Helper method to convert a vector of doubles
 */
XmlRpc::XmlRpcValue vectorToXMLRPC(const std::vector<double>& array)
{
  XmlRpc::XmlRpcValue xml;
  xml.setSize(array.size());
  for (unsigned int i = 0; i < array.size(); ++i)
  {
    xml[i] = array[i];
  }
  return xml;
}

XmlRpc::XmlRpcValue polygonToXMLRPC(const nav_2d_msgs::Polygon2D& polygon, bool array_of_arrays)
{
  XmlRpc::XmlRpcValue xml;
  if (array_of_arrays)
  {
    xml.setSize(polygon.points.size());
    for (unsigned int i = 0; i < polygon.points.size(); ++i)
    {
      xml[i].setSize(2);
      const Point2D& p = polygon.points[i];
      xml[i][0] = p.x;
      xml[i][1] = p.y;
    }
  }
  else
  {
    std::vector<double> xs, ys;
    polygonToParallelArrays(polygon, xs, ys);
    xml["x"] = vectorToXMLRPC(xs);
    xml["y"] = vectorToXMLRPC(ys);
  }
  return xml;
}

void polygonToParams(const nav_2d_msgs::Polygon2D& polygon, const ros::NodeHandle& nh, const std::string parameter_name,
                     bool array_of_arrays)
{
  nh.setParam(parameter_name, polygonToXMLRPC(polygon, array_of_arrays));
}

nav_2d_msgs::Polygon2D polygonFromParallelArrays(const std::vector<double>& xs, const std::vector<double>& ys)
{
  if (xs.size() < 3)
  {
    throw PolygonParseException("You must specify at least three points for the polygon.");
  }
  else if (xs.size() != ys.size())
  {
    throw PolygonParseException("Length of x array does not match length of y array.");
  }

  Polygon2D polygon;
  polygon.points.resize(xs.size());
  for (unsigned int i = 0; i < xs.size(); i++)
  {
    Point2D& pt = polygon.points[i];
    pt.x = xs[i];
    pt.y = ys[i];
  }
  return polygon;
}

void polygonToParallelArrays(const nav_2d_msgs::Polygon2D polygon, std::vector<double>& xs, std::vector<double>& ys)
{
  xs.clear();
  ys.clear();
  for (const Point2D& pt : polygon.points)
  {
    xs.push_back(pt.x);
    ys.push_back(pt.y);
  }
}

bool equals(const nav_2d_msgs::Polygon2D& polygon0, const nav_2d_msgs::Polygon2D& polygon1)
{
  if (polygon0.points.size() != polygon1.points.size())
  {
    return false;
  }
  for (unsigned int i=0; i < polygon0.points.size(); i++)
  {
    if (polygon0.points[i].x != polygon1.points[i].x ||polygon0.points[i].y != polygon1.points[i].y)
    {
      return false;
    }
  }
  return true;
}

nav_2d_msgs::Polygon2D movePolygonToPose(const nav_2d_msgs::Polygon2D& polygon,
                                         const geometry_msgs::Pose2D& pose)
{
  nav_2d_msgs::Polygon2D new_polygon;
  new_polygon.points.resize(polygon.points.size());
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);
  for (unsigned int i = 0; i < polygon.points.size(); ++i)
  {
    nav_2d_msgs::Point2D& new_pt = new_polygon.points[i];
    new_pt.x = pose.x + polygon.points[i].x * cos_th - polygon.points[i].y * sin_th;
    new_pt.y = pose.y + polygon.points[i].x * sin_th + polygon.points[i].y * cos_th;
  }
  return new_polygon;
}

bool isInside(const nav_2d_msgs::Polygon2D& polygon, const double x, const double y)
{
  // Determine if the given point is inside the polygon using the number of crossings method
  // https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
  int n = polygon.points.size();
  int cross = 0;
  // Loop from i = [0 ... n - 1] and j = [n - 1, 0 ... n - 2]
  // Ensures first point connects to last point
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    // Check if the line to x,y crosses this edge
    if ( ((polygon.points[i].y > y) != (polygon.points[j].y > y))
           && (x < (polygon.points[j].x - polygon.points[i].x) * (y - polygon.points[i].y) /
            (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x) )
    {
      cross++;
    }
  }
  // Return true if the number of crossings is odd
  return cross % 2 > 0;
}

void calculateMinAndMaxDistances(const nav_2d_msgs::Polygon2D& polygon, double& min_dist, double& max_dist)
{
  min_dist = std::numeric_limits<double>::max();
  max_dist = 0.0;
  auto& points = polygon.points;
  if (points.size() == 0)
  {
    return;
  }

  for (unsigned int i = 0; i < points.size() - 1; ++i)
  {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = hypot(points[i].x, points[i].y);
    double edge_dist = distanceToLine(0.0, 0.0, points[i].x, points[i].y,
                                      points[i + 1].x, points[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist = hypot(points.back().x, points.back().y);
  double edge_dist = distanceToLine(0.0, 0.0, points.back().x, points.back().y,
                                      points.front().x, points.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}
}  // namespace nav_2d_utils

// Adapt Point2D for use with the triangulation library
namespace mapbox
{
namespace util
{
template <>
struct nth<0, nav_2d_msgs::Point2D>
{
  inline static double get(const nav_2d_msgs::Point2D& point)
  {
    return point.x;
  };
};

template <>
struct nth<1, nav_2d_msgs::Point2D>
{
  inline static double get(const nav_2d_msgs::Point2D& point)
  {
    return point.y;
  };
};

}  // namespace util
}  // namespace mapbox


namespace nav_2d_utils
{
std::vector<nav_2d_msgs::Point2D> triangulate(const nav_2d_msgs::ComplexPolygon2D& polygon)
{
  // Compute the triangulation
  std::vector<std::vector<nav_2d_msgs::Point2D>> rings;
  rings.reserve(1 + polygon.inner.size());
  rings.push_back(polygon.outer.points);
  for (const nav_2d_msgs::Polygon2D& inner : polygon.inner)
  {
    rings.push_back(inner.points);
  }
  std::vector<unsigned int> indices = mapbox::earcut<unsigned int>(rings);

  // Create a sequential point index. The triangulation results are indices in this vector.
  std::vector<nav_2d_msgs::Point2D> points;
  for (const auto& ring : rings)
  {
    for (const nav_2d_msgs::Point2D& point : ring)
    {
      points.push_back(point);
    }
  }

  // Construct the output triangles
  std::vector<nav_2d_msgs::Point2D> result;
  result.reserve(indices.size());
  for (unsigned int index : indices)
  {
    result.push_back(points[index]);
  }
  return result;
}

std::vector<nav_2d_msgs::Point2D> triangulate(const nav_2d_msgs::Polygon2D& polygon)
{
  nav_2d_msgs::ComplexPolygon2D complex;
  complex.outer = polygon;
  return triangulate(complex);
}


}  // namespace nav_2d_utils
