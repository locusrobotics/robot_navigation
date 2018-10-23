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

#include <ros/ros.h>
#include <nav_grid_iterators/iterators.h>
#include <nav_grid/vector_nav_grid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_2d_utils/conversions.h>
#include <vector>

template<class Iterator>
class InfiniteIterator
{
public:
  InfiniteIterator(Iterator it, unsigned char active_value, unsigned char seen_value)
    : it_(it), active_(active_value), seen_(seen_value)
  {
  }

  void iterate(nav_grid::NavGrid<unsigned char>& grid)
  {
    // Note that this demo assumes that each iterator has at least one point on the grid
    grid.setValue(*it_, seen_);
    ++it_;
    if (it_ == it_.end())
    {
      it_ = it_.begin();
    }
    grid.setValue(*it_, active_);
  }

private:
  Iterator it_;
  unsigned char active_, seen_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iterator_demo");
  nav_grid::VectorNavGrid<unsigned char> grid;
  nav_grid::NavGridInfo info;
  sleep(5.0);
  info.width = 75;
  info.height = 60;
  info.resolution = 0.1;
  grid.setInfo(info);

  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
  ros::Rate r(33);

  nav_2d_msgs::Polygon2D triangle;
  triangle.points.resize(3);
  triangle.points[0].x = 1.9;
  triangle.points[0].y = 0.5;
  triangle.points[1].x = 3.5;
  triangle.points[1].y = 1.0;
  triangle.points[2].x = 2.0;
  triangle.points[2].y = 2.4;

  nav_2d_msgs::Polygon2D diamond;
  diamond.points.resize(4);
  diamond.points[0].x = 4.5;
  diamond.points[0].y = 0.0;
  diamond.points[1].x = 5.3;
  diamond.points[1].y = 1.25;
  diamond.points[2].x = 4.5;
  diamond.points[2].y = 2.5;
  diamond.points[3].x = 3.7;
  diamond.points[3].y = 1.25;

  nav_2d_msgs::Polygon2D diamond2;
  diamond2.points.resize(4);
  diamond2.points[0].x = 6.5;
  diamond2.points[0].y = 0.0;
  diamond2.points[1].x = 7.3;
  diamond2.points[1].y = 1.25;
  diamond2.points[2].x = 6.5;
  diamond2.points[2].y = 2.5;
  diamond2.points[3].x = 5.7;
  diamond2.points[3].y = 1.25;

  InfiniteIterator<nav_grid_iterators::WholeGrid> whole_grid(nav_grid_iterators::WholeGrid(&info), 100, 50);
  InfiniteIterator<nav_grid_iterators::SubGrid> sub_grid(nav_grid_iterators::SubGrid(&info, 3, 5, 14, 15), 99, 101);
  InfiniteIterator<nav_grid_iterators::Line> line(nav_grid_iterators::Line(&info, 7.3, 2.8, 0.2, 2.5), 126, 200);
  InfiniteIterator<nav_grid_iterators::Line> line2(nav_grid_iterators::Line(&info, 7.3, 3.3, 0.2, 3.0, true, false),
                                                   126, 200);
  InfiniteIterator<nav_grid_iterators::CircleFill> circle_fill(nav_grid_iterators::CircleFill(&info, 1.25, 4.8, 1.0),
                                                               254, 255);
  InfiniteIterator<nav_grid_iterators::Spiral> spiral(nav_grid_iterators::Spiral(&info, 3.75, 4.8, 1.0), 254, 126);
  InfiniteIterator<nav_grid_iterators::CircleOutline> circle_o(nav_grid_iterators::CircleOutline(&info, 6.25, 4.8, 1.0),
                                                               254, 99);
  InfiniteIterator<nav_grid_iterators::PolygonFill> poly_f(nav_grid_iterators::PolygonFill(&info, triangle), 126, 254);
  InfiniteIterator<nav_grid_iterators::PolygonOutline> poly_o(nav_grid_iterators::PolygonOutline(&info, diamond), 0, 1);
  InfiniteIterator<nav_grid_iterators::PolygonOutline> poly_r(nav_grid_iterators::PolygonOutline(&info, diamond2, 0),
                                                              0, 1);

  nav_msgs::OccupancyGrid ogrid;
  ogrid.header.frame_id = info.frame_id;
  ogrid.info = nav_2d_utils::infoToInfo(info);
  ogrid.data.resize(info.width * info.height);

  while (ros::ok())
  {
    whole_grid.iterate(grid);
    sub_grid.iterate(grid);
    line.iterate(grid);
    line2.iterate(grid);
    circle_fill.iterate(grid);
    spiral.iterate(grid);
    circle_o.iterate(grid);
    poly_f.iterate(grid);
    poly_o.iterate(grid);
    poly_r.iterate(grid);

    // Manaully creating OccupancyGrid (rather than use nav_grid_pub_sub) to avoid circular dependency
    ogrid.header.stamp = ros::Time::now();

    unsigned int data_index = 0;
    for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
    {
      ogrid.data[data_index++] = grid(index);
    }
    pub.publish(ogrid);
    r.sleep();
  }

  return 0;
}
