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

#ifndef COSTMAP_QUEUE_COSTMAP_QUEUE_H
#define COSTMAP_QUEUE_COSTMAP_QUEUE_H

#include <nav_core2/costmap.h>
#include <costmap_queue/map_based_queue.h>
#include <nav_grid/vector_nav_grid.h>
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace costmap_queue
{
/**
 * @class CellData
 * @brief Storage for cell information used during queue expansion
 */
class CellData
{
public:
  /**
   * @brief Real Constructor
   * @param d The distance to the nearest obstacle
   * @param x The x coordinate of the cell in the cost map
   * @param y The y coordinate of the cell in the cost map
   * @param sx The x coordinate of the closest source cell in the costmap
   * @param sy The y coordinate of the closest source cell in the costmap
   */
  CellData(const double d, const unsigned int x, const unsigned int y, const unsigned int sx, const unsigned int sy) :
    distance_(d), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }

  /**
   * @brief Default Constructor - Should be used sparingly
   */
  CellData() :
    distance_(std::numeric_limits<double>::max()), x_(0), y_(0), src_x_(0), src_y_(0)
  {
  }

  double distance_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @class CostmapQueue
 * @brief A tool for finding the cells closest to some set of originating cells.
 *
 * A common operation with costmaps is to define a set of cells in the costmap, and then
 * perform some operation on all the other cells based on which cell in the original set
 * the other cells are closest to. This operation is done in the inflation layer to figure out
 * how far each cell is from an obstacle, and is also used in a number of Trajectory cost functions.
 *
 * It is implemented with a queue. The standard operation is to enqueueCell the original set, and then
 * retreive the other cells with the isEmpty/getNextCell iterator-like functionality. getNextCell
 * returns an object that contains the coordinates of this cell and the origin cell, as well as
 * the distance between them. By default, the Euclidean distance is used for ordering, but passing in
 * manhattan=true to the constructor will use the Manhattan distance.
 *
 * The validCellToQueue overridable-function allows for deriving classes to limit the queue traversal
 * to a subset of all costmap cells. LimitedCostmapQueue does this by ignoring distances above a limit.
 *
 */
class CostmapQueue : public MapBasedQueue<CellData>
{
public:
  /**
   * @brief constructor
   * @param costmap Costmap which defines the size/number of cells
   * @param manhattan If true, sort cells by Manhattan distance, otherwise use Euclidean distance
   */
  explicit CostmapQueue(nav_core2::Costmap& costmap, bool manhattan = false);

  /**
   * @brief Clear the queue
   */
  void reset() override;

  /**
   * @brief Add a cell the queue
   * @param x X coordinate of the cell
   * @param y Y coordinate of the cell
   */
  void enqueueCell(unsigned int x, unsigned int y);

  /**
   * @brief Get the next cell to examine, and enqueue its neighbors as needed
   * @return The next cell
   *
   * NB: Assumes that isEmpty has been called before this call and returned false
   */
  CellData getNextCell();

  /**
   * @brief Get the maximum x or y distance we'll need to calculate the distance between
   */
  virtual int getMaxDistance() const { return std::max(costmap_.getWidth(), costmap_.getHeight()); }

  /**
   * @brief Check to see if we should add this cell to the queue. Always true unless overridden.
   * @param cell The cell to check
   * @return True, unless overriden
   */
  virtual bool validCellToQueue(const CellData& cell) { return true; }

  /**
   * @brief convenience definition for a pointer
   */
  using Ptr = std::shared_ptr<CostmapQueue>;
protected:
  /**
   * @brief Enqueue a cell with the given coordinates and the given source cell
   */
  void enqueueCell(unsigned int cur_x, unsigned int cur_y, unsigned int src_x, unsigned int src_y);

  /**
   * @brief Compute the cached distances
   */
  void computeCache();

  nav_core2::Costmap& costmap_;

  // This really should be VectorNavGrid<bool>, but since
  // vector<bool> is wacky, it would result in compile errors.
  nav_grid::VectorNavGrid<unsigned char> seen_;
  bool manhattan_;
protected:
  /**
   * @brief  Lookup pre-computed distances
   * @param cur_x The x coordinate of the current cell
   * @param cur_y The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return Precomputed distance
   *
   * NB: Note that while abs() has the correct behavior i.e. abs(2 - 5) ==> 3.
   *     std::abs() without explicit casting does not have the correct behavior
   *     i.e. std::abs(2 - 5) ==> (the unsigned version of) -3.
   *     We're using explicit casting here to ensure the behavior is not compiler dependent.
   *     std::abs(static_cast<int>(2) - static_cast<int>(5)) ==> 3.
   */
  inline double distanceLookup(const unsigned int cur_x, const unsigned int cur_y,
                               const unsigned int src_x, const unsigned int src_y)
  {
    unsigned int dx = std::abs(static_cast<int>(cur_x) - static_cast<int>(src_x));
    unsigned int dy = std::abs(static_cast<int>(cur_y) - static_cast<int>(src_y));
    return cached_distances_[dx][dy];
  }
  std::vector<std::vector<double> > cached_distances_;
  int cached_max_distance_;
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE_COSTMAP_QUEUE_H
