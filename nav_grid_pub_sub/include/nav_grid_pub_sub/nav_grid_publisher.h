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

#ifndef NAV_GRID_PUB_SUB_NAV_GRID_PUBLISHER_H
#define NAV_GRID_PUB_SUB_NAV_GRID_PUBLISHER_H

#include <nav_grid_pub_sub/nav_grid_message_utils.h>
#include <nav_grid_pub_sub/occ_grid_message_utils.h>
#include <nav_grid_pub_sub/cost_interpretation.h>
#include <nav_grid_pub_sub/cost_interpretation_tables.h>
#include <ros/ros.h>
#include <nav_grid/nav_grid.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/bounds.h>
#include <geometry_msgs/PolygonStamped.h>
#include <limits>
#include <string>
#include <vector>

namespace nav_grid_pub_sub
{
// Helper function: needs temp double variables for conversion, then made into floats for Point32
inline geometry_msgs::Point32 makePoint(const nav_grid::NavGridInfo& info, int x, int y)
{
  double point_x = 0.0;
  double point_y = 0.0;
  gridToWorld(info, x, y, point_x, point_y);
  geometry_msgs::Point32 point;
  point.x = point_x;
  point.y = point_y;
  return point;
}

/**
 * @class GenericGridPublisher
 * @brief An interface for publishing NavGridOfX/OccupancyGrid msgs and their updates periodically
 *
 * This class can potentially publish on five different messages on five topics
 *     * NavGridOfX / grid
 *     * NavGridOfXUpdate / grid_update
 *     * OccupancyGrid / costmap
 *     * OccupancyGridUpdate / costmap_updates
 *     * PolygonStamped / update_area
 * (where X is either Chars or Doubles)
 * These names can be overridden, and if you wish to not publish a portion, you can set the
 * topic name to the empty string in the init method.
 *
 * The main reason for the two types of publishers (NavGridOfX vs OccupancyGrid) is that the standard
 * usage for OccupancyGrid has been that values are ranged [0, 100] with -1/255 reserved for unknown.
 * Implementing classes need to find a way to convert the full range of values to fit that range.
 * NavGridOfChars does not follow that convention, so all values are used, and NavGridOfDoubles does not
 * need to limit to the 256 values of OccupancyGrid. However, there are legacy applications that like the
 * OccupancyGrid message, so we maintain the ability to publish both.
 *
 * Full Grid vs. Update Publishing: If you just call publish() with no params, this class will only
 * publish the full grid messages (NavGridOfX/OccupancyGrid). If you call publish(...) with the
 * bounds parameter, this class will publish the full grid messages if publish_updates was set to false
 * during init or the NavGridInfo has changed (i.e. the size/origin/resolution/frame has changed).
 * Otherwise, it will publish the update messages (NavGridOfXUpdate/OccupancyGridUpdate) and
 * the update_area polygon.
 *
 * How often the full grid is published is controlled by the full_publish_cycle and how often you call
 * one of the publish methods.
 *      * If the full_publish_cycle is 0, which is the default, the full grid will be published
 *        every time you call publish.
 *      * If the full_publish_cycle is negative, you can avoid publishing all together.
 *      * Otherwise, the full grid will only be published when publish is called if `full_publish_cycle` time
 *        has passed since the last full grid publish.
 *      * Note that full grids may also be published when attempting to publish an update but the grid info has changed
 * You can control how often updates are published with similar logic and the update_publish_cycle argument. If the
 * update_publish_cycle is positive, the Bounds from successive calls will be merged so the resulting update will
 * cover the superset of all the bounds.
 */
template<typename NumericType, typename NavGridOfX, typename NavGridOfXUpdate>
class GenericGridPublisher
{
public:
  explicit GenericGridPublisher(nav_grid::NavGrid<NumericType>& data) : data_(data) {}

  /**
   * @brief Initialize method for determining what gets published when
   * @param nh NodeHandle used for creating publishers
   * @param grid Data source
   * @param nav_grid_topic Topic to publish the NavGridOfX on. If empty, don't publish that message.
   * @param occupancy_grid_topic Topic to publish the OccupancyGrid on. If empty, don't publish that message.
   * @param update_area_topic Topic to publish the update area polygon on. If empty, don't publish that message.
   * @param publish_updates If true, publishes _update topics. If false, always publish full grid.
   * @param full_publish_cycle If positive, limits how often the full grid is published. If negative, never publishes.
   * @param update_publish_cycle If positive, limits how often the update is published. If negative, never publishes.
   *
   *
   * Note: We use the callback functions for when the publisher gets a new subscription to ensure the new subscriber
   * gets the most up to date data. If we just latched the topic, several updates may have occurred between the first
   * whole grid publication and the new subscription, so it would have out-of-date information.
   */
  void init(ros::NodeHandle& nh,
            const std::string& nav_grid_topic = "grid",
            const std::string& occupancy_grid_topic = "costmap",
            const std::string& update_area_topic = "update_area",
            bool publish_updates = true,
            ros::Duration full_publish_cycle = ros::Duration(0),
            ros::Duration update_publish_cycle = ros::Duration(0))
  {
    full_publish_cycle_ = full_publish_cycle;
    update_publish_cycle_ = update_publish_cycle;
    last_full_publish_ = ros::Time(0);
    last_update_publish_ = ros::Time(0);
    publish_updates_ = publish_updates;

    createPublishers<NavGridOfX, NavGridOfXUpdate>(
        nh, nav_grid_topic, boost::bind(&GenericGridPublisher::onNewSubscriptionNav, this, _1),
        nav_pub_, nav_update_pub_, publish_updates);

    createPublishers<nav_msgs::OccupancyGrid, map_msgs::OccupancyGridUpdate>(
        nh, occupancy_grid_topic, boost::bind(&GenericGridPublisher::onNewSubscriptionOcc, this, _1),
        occ_pub_, occ_update_pub_, publish_updates);

    if (update_area_topic.length() > 0)
    {
      update_area_pub_ = nh.advertise<geometry_msgs::PolygonStamped>(update_area_topic, 1);
    }
  }

  /**
   * @brief Publish the full grid if the full_publish_cycle allows
   */
  void publish()
  {
    if (!shouldPublishFull()) return;
    last_full_publish_ = ros::Time::now();
    synced_time_stamp_ = last_full_publish_;
    publishNav();
    publishOcc();
  }

  /**
   * @brief Publish the full grid or updates, as dictated by parameters passed to init
   *
   * The bounds provided are of the form [min, max] i.e. the maximum is included in the range.
   * If either range is empty, that indicates none of the grid has been updated, but we still may want
   * to publish some of the data.
   */
  void publish(const nav_core2::UIntBounds& bounds)
  {
    if (!publish_updates_)
    {
      // Don't publish an update, publish the full grid if enough time has passed
      publish();
      return;
    }

    update_bounds_.merge(bounds);

    if (!shouldPublishUpdate()) return;

    const nav_grid::NavGridInfo& info = data_.getInfo();
    last_update_publish_ = ros::Time::now();
    synced_time_stamp_ = last_update_publish_;

    if (saved_info_ != info)
    {
      // If the info has changed, force publish the whole grid
      saved_info_ = info;
      publishNav();
      publishOcc();
    }
    else if (!update_bounds_.isEmpty())
    {
      // If actual data was updated, publish the updates
      publishNavUpdate(update_bounds_);
      publishOccUpdate(update_bounds_);
    }

    // Publish the update area (or an empty polygon message if there was no update)
    publishUpdateArea(info, update_bounds_);

    update_bounds_.reset();
  }


protected:
  template<class FullGridType, class UpdateType, class Callback>
  void createPublishers(ros::NodeHandle& nh, const std::string& topic, Callback new_subscription_callback,
                        ros::Publisher& full_grid_pub, ros::Publisher& update_pub, bool publish_updates)
  {
    if (topic.length() > 0)
    {
      full_grid_pub = nh.advertise<FullGridType>(topic, 1, new_subscription_callback);
      if (publish_updates)
      {
        update_pub = nh.advertise<UpdateType>(topic + "_updates", 1);
      }
    }
  }

  virtual nav_msgs::OccupancyGrid toOccupancyGrid(const ros::Time& timestamp) = 0;
  virtual map_msgs::OccupancyGridUpdate toOccupancyGridUpdate(const nav_core2::UIntBounds& bounds,
                                                              const ros::Time& timestamp) = 0;

  bool shouldPublishHelper(const ros::Time& last_publish, const ros::Duration& cycle) const
  {
    double cycle_secs = cycle.toSec();
    if (cycle_secs < 0.0)
    {
      return false;
    }
    else if (cycle_secs == 0.0)
    {
      return true;
    }
    else
    {
      return last_publish + cycle < ros::Time::now();
    }
  }

  inline bool shouldPublishFull() const
  {
    return shouldPublishHelper(last_full_publish_, full_publish_cycle_);
  }

  inline bool shouldPublishUpdate() const
  {
    return shouldPublishHelper(last_update_publish_, update_publish_cycle_);
  }

  void publishUpdateArea(const nav_grid::NavGridInfo& info, const nav_core2::UIntBounds& bounds)
  {
    if (update_area_pub_.getNumSubscribers() == 0) return;

    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = info.frame_id;
    polygon.header.stamp = synced_time_stamp_;

    if (!bounds.isEmpty())
    {
      polygon.polygon.points.push_back(makePoint(info, bounds.getMinX(), bounds.getMinY()));
      polygon.polygon.points.push_back(makePoint(info, bounds.getMaxX(), bounds.getMinY()));
      polygon.polygon.points.push_back(makePoint(info, bounds.getMaxX(), bounds.getMaxY()));
      polygon.polygon.points.push_back(makePoint(info, bounds.getMinX(), bounds.getMaxY()));
    }
    update_area_pub_.publish(polygon);
  }

  void onNewSubscriptionNav(const ros::SingleSubscriberPublisher& pub)
  {
    pub.publish(toMsg(data_, ros::Time::now()));
  }

  void onNewSubscriptionOcc(const ros::SingleSubscriberPublisher& pub)
  {
    pub.publish(toOccupancyGrid(ros::Time::now()));
  }

  void publishNav()
  {
    if (nav_pub_.getNumSubscribers() == 0) return;
    nav_pub_.publish(toMsg(data_, synced_time_stamp_));
  }

  void publishOcc()
  {
    if (occ_pub_.getNumSubscribers() == 0) return;
    occ_pub_.publish(toOccupancyGrid(synced_time_stamp_));
  }

  void publishNavUpdate(const nav_core2::UIntBounds& bounds)
  {
    if (nav_update_pub_.getNumSubscribers() == 0) return;
    nav_update_pub_.publish(nav_grid_pub_sub::toUpdate(data_, bounds, synced_time_stamp_));
  }

  void publishOccUpdate(const nav_core2::UIntBounds& bounds)
  {
    if (occ_update_pub_.getNumSubscribers() == 0) return;
    occ_update_pub_.publish(toOccupancyGridUpdate(bounds, synced_time_stamp_));
  }

  // Data
  nav_grid::NavGrid<NumericType>& data_;
  nav_grid::NavGridInfo saved_info_;
  bool publish_updates_;

  // Track time
  ros::Duration full_publish_cycle_, update_publish_cycle_;
  ros::Time last_full_publish_, last_update_publish_, synced_time_stamp_;

  // Track Update Bounds
  nav_core2::UIntBounds update_bounds_;

  // Publishers
  ros::Publisher nav_pub_, nav_update_pub_,
                 occ_pub_, occ_update_pub_,
                 update_area_pub_;
};

/**
 * @class NavGridPublisher
 * @brief An interface for publishing NavGridOfChars/OccupancyGrid msgs and their updates periodically
 *
 * Uses the cost_interpretation_table_ to define how to translate to OccupancyGrid messages
 */
class NavGridPublisher
  : public GenericGridPublisher<unsigned char, nav_2d_msgs::NavGridOfChars, nav_2d_msgs::NavGridOfCharsUpdate>
{
public:
  using GenericGridPublisher<unsigned char, nav_2d_msgs::NavGridOfChars, nav_2d_msgs::NavGridOfCharsUpdate>
        ::GenericGridPublisher;

  void setCostInterpretation(const std::vector<unsigned char>& cost_interpretation_table)
  {
    cost_interpretation_table_ = cost_interpretation_table;
  }
protected:
  nav_msgs::OccupancyGrid toOccupancyGrid(const ros::Time& timestamp) override
  {
    return nav_grid_pub_sub::toOccupancyGrid(data_, timestamp, cost_interpretation_table_);
  }

  map_msgs::OccupancyGridUpdate toOccupancyGridUpdate(const nav_core2::UIntBounds& bounds,
                                                      const ros::Time& timestamp) override
  {
    return nav_grid_pub_sub::toOccupancyGridUpdate(data_, bounds, timestamp, cost_interpretation_table_);
  }
  std::vector<unsigned char> cost_interpretation_table_ { OCC_GRID_PUBLISHING };
};

/**
 * @class ScaleGridPublisher
 * @brief An interface for publishing NavGridOfDoubles/OccupancyGrid msgs and their updates periodically
 */
template<typename NumericType>
class ScaleGridPublisher
  : public GenericGridPublisher<NumericType, nav_2d_msgs::NavGridOfDoubles, nav_2d_msgs::NavGridOfDoublesUpdate>
{
public:
  using GenericGridPublisher<NumericType, nav_2d_msgs::NavGridOfDoubles, nav_2d_msgs::NavGridOfDoublesUpdate>
        ::GenericGridPublisher;

  void setIgnoreValue(NumericType ignore_value) { ignore_value_ = ignore_value; }

protected:
  nav_msgs::OccupancyGrid toOccupancyGrid(const ros::Time& timestamp) override
  {
    nav_grid_pub_sub::getExtremeValues(this->data_, ignore_value_, min_val_, max_val_);
    return nav_grid_pub_sub::toOccupancyGrid(this->data_, min_val_, max_val_, ignore_value_, timestamp);
  }

  map_msgs::OccupancyGridUpdate toOccupancyGridUpdate(const nav_core2::UIntBounds& bounds,
                                                      const ros::Time& timestamp) override
  {
    return nav_grid_pub_sub::toOccupancyGridUpdate(this->data_, bounds, min_val_, max_val_, ignore_value_, timestamp);
  }

  NumericType ignore_value_ { std::numeric_limits<NumericType>::max() };
  NumericType min_val_, max_val_;
};


}  // namespace nav_grid_pub_sub

#endif  // NAV_GRID_PUB_SUB_NAV_GRID_PUBLISHER_H
