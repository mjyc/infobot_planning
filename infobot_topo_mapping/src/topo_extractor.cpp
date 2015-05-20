#include "infobot_topo_mapping/topo_extractor.h"

#include <math.h>
#include <boost/format.hpp>
#include <ros/console.h>
#include <string>
#include <vector>

#include "infobot_topo_mapping/visualizer.h"
#include "infobot_topo_mapping/topo_exception.h"

namespace infobot_topo_mapping
{

using infobot_topo_msgs::TopologicalMap;
using infobot_topo_msgs::TopologicalPlace;
using infobot_topo_msgs::TopologicalView;
using infobot_topo_msgs::MetricMapTransform;
using nav_msgs::OccupancyGrid;
using geometry_msgs::Pose2D;
using geometry_msgs::Pose;
using std::vector;
using boost::format;
using boost::str;


// -----------------------------------------------------------------------------
void TopoExtractor::processMetricMap(const OccupancyGrid &metric_map,
                                     const Pose2D &seed_view)
{
  // Clear the existing topo map
  clearTopoMap();

  // Calculate the transforms of the metric map
  const Pose &origin = metric_map.info.origin;
  tf2::Quaternion origin_q(origin.orientation.x, origin.orientation.y,
                           origin.orientation.z, origin.orientation.w);
  tf2::Vector3 origin_v(origin.position.x, origin.position.y,
                        origin.position.z);
  tf2::Transform transform(origin_q, origin_v);
  tf2::Transform transform_inv = transform.inverse();

  // Get the seed as matrix point
  cv::Point seed_point = getMatPoint(metric_map, transform_inv, seed_view);

  // Convert occupancy grid to matrix
  ROS_INFO("Calculating goodness / cost map.");
  cv::Mat occupancy_mat = getOccupancyMatrix(metric_map);
  visualizer_.showMatrix(3, "OccupancyGrid", occupancy_mat);

  // Get the goodness probability map
  cv::Mat goodness_map = cost_extractor_.getProbabilityMap(occupancy_mat,
                                                           metric_map.info.resolution,
                                                           seed_point);

  // Wait for a key if we are showing visualizations
  if (!visualizer_.waitForKey())
    throw TopoException("Interrupted!");

  // Sample places
  ROS_INFO("Sampling places.");
  vector<cv::Point> seed_points;
  seed_points.push_back(seed_point);
  cv::Mat place_map = place_sampler_.samplePlaces(goodness_map,
                                                  occupancy_mat,
                                                  seed_points,
                                                  metric_map.info.resolution);

  // Extract the place locations from the place map and create a topo map
  topo_map_->topo_map_id = "real-floor4";
  topo_map_->header.seq = 1;
  topo_map_->header.stamp = ros::Time::now();
  topo_map_->header.frame_id = "map";  // This just helps when displaying things in RViz
  string metric_map_id = "real-floor4";
  uint32_t place_id = 1;
  uint32_t view_id = 1;
  for (int y = 0; y < place_map.rows; ++y)
    for (int x = 0; x < place_map.cols; ++x)
    {
      if (place_map.at<uint8_t>(y, x) > 0)
      {
        // FIX: Get the correct ID
        // There is a place at x, y
        TopologicalPlace place;
        place.metric_map_id = metric_map_id;
        place.topo_map_id = topo_map_->topo_map_id;
        place.place_id = place_id++;

        // Generate views
        for (int a = 0; a < 360; a += 30)
        {
          double r =  static_cast<double>(a) * M_PI / 180.0;
          TopologicalView view;
          view.metric_map_id = metric_map_id;
          view.topo_map_id = topo_map_->topo_map_id;
          view.view_id = view_id++;
          view.pose =  getMapPose(metric_map, transform, x, y, r);
          place.views.push_back(view);
        }
        topo_map_->places.push_back(place);
      }
    }
}


// -----------------------------------------------------------------------------
void TopoExtractor::processMetricMap(const OccupancyGrid &metric_map,
                                     const vector<MetricMapTransform> &map_relations)
{
  ROS_INFO("Adding to existing map with %d palces", (int)topo_map_->places.size());

  // Calculate the transforms of the metric map
  const Pose &origin = metric_map.info.origin;
  tf2::Quaternion origin_q(origin.orientation.x, origin.orientation.y,
                           origin.orientation.z, origin.orientation.w);
  tf2::Vector3 origin_v(origin.position.x, origin.position.y,
                        origin.position.z);
  tf2::Transform transform(origin_q, origin_v);
  tf2::Transform transform_inv = transform.inverse();

  // Process existing topo map
  vector<cv::Point> seed_points;
  int max_place_id = 0;
  int max_view_id = 0;
  for (int i=0; i < topo_map_->places.size(); ++i)
  {
    const TopologicalPlace &place = topo_map_->places[i];
    if (place.place_id > max_place_id)
    {
      max_place_id = place.place_id;
    }
    for (int j=0; j < place.views.size(); ++j)
    {
      if (place.views[j].view_id > max_view_id)
      {
        max_view_id = place.views[j].view_id;
      }
    }

    // Get the seed as matrix point
    cv::Point seed_point = getMatPoint(metric_map, transform_inv, place.views[0].pose);
    seed_points.push_back(seed_point);
  }

  // Convert occupancy grid to matrix
  ROS_INFO("Calculating goodness / cost map.");
  cv::Mat occupancy_mat = getOccupancyMatrix(metric_map);
  visualizer_.showMatrix(3, "OccupancyGrid", occupancy_mat);

  // Get the goodness probability map
  cv::Mat goodness_map = cost_extractor_.getProbabilityMap(occupancy_mat,
                                                           metric_map.info.resolution,
                                                           seed_points[0]);

  // Wait for a key if we are showing visualizations
  if (!visualizer_.waitForKey())
    throw TopoException("Interrupted!");

  // Sample places
  ROS_INFO("Sampling places.");
  cv::Mat place_map = place_sampler_.samplePlaces(goodness_map,
                                                  occupancy_mat,
                                                  seed_points,
                                                  metric_map.info.resolution);

  // Extract the place locations from the place map and create a topo map
  topo_map_->header.seq++;
  topo_map_->header.stamp = ros::Time::now();
  topo_map_->header.frame_id = "map";  // This just helps when displaying things in RViz
  string metric_map_id = "real-floor4";
  uint32_t place_id = max_place_id + 1;
  uint32_t view_id = max_view_id + 1;

  for (int y = 0; y < place_map.rows; ++y)
    for (int x = 0; x < place_map.cols; ++x)
    {
      if ((place_map.at<uint8_t>(y, x) > 0) && (!inPoints(seed_points, x, y)))
      {
        // FIX: Get the correct ID
        // There is a place at x, y
        TopologicalPlace place;
        place.metric_map_id = metric_map_id;
        place.topo_map_id = topo_map_->topo_map_id;
        place.place_id = place_id++;

        // Generate views
        for (int a = 0; a < 360; a += 30)
        {
          double r =  static_cast<double>(a) * M_PI / 180.0;
          TopologicalView view;
          view.metric_map_id = metric_map_id;
          view.topo_map_id = topo_map_->topo_map_id;
          view.view_id = view_id++;
          view.pose =  getMapPose(metric_map, transform, x, y, r);
          place.views.push_back(view);
        }
        topo_map_->places.push_back(place);
      }
    }
}


// -----------------------------------------------------------------------------
void TopoExtractor::clearTopoMap()
{
  topo_map_->header.seq = 0;
  topo_map_->header.stamp.sec = 0;
  topo_map_->header.stamp.nsec = 0;
  topo_map_->header.frame_id = "0";
  topo_map_->topo_map_id = "";
  topo_map_->places.clear();
}


// -----------------------------------------------------------------------------
cv::Point TopoExtractor::getMatPoint(const OccupancyGrid &metric_map,
                                     const tf2::Transform &origin_transform,
                                     const Pose2D &pose)
{
  tf2::Vector3 pose_v(pose.x, pose.y, 0.0);
  tf2::Vector3 pose_tr = origin_transform(pose_v);

  // Map's (0, 0) is in it's bottom left corner
  // Matrix (0, 0) is in it's upper left corner
  return cv::Point(round(pose_tr.x() / metric_map.info.resolution),
                   metric_map.info.height - round(pose_tr.y() / metric_map.info.resolution));
}


// -----------------------------------------------------------------------------
Pose2D TopoExtractor::getMapPose(const OccupancyGrid &metric_map,
                                 const tf2::Transform &origin_transform,
                                 int x, int y, double angle)
{
  // Map's (0, 0) is in it's bottom left corner
  // Matrix (0, 0) is in it's upper left corner
  y = metric_map.info.height - y;

  // Convert to meters
  double x_m = x * metric_map.info.resolution;
  double y_m = y * metric_map.info.resolution;

  // Transform
  tf2::Vector3 pose_v = origin_transform(tf2::Vector3(x_m, y_m, 0.0));

  // Set
  Pose2D pose;
  pose.x = pose_v.x();
  pose.y = pose_v.y();
  pose.theta = angle;

  return pose;
}


// -----------------------------------------------------------------------------
cv::Mat TopoExtractor::getOccupancyMatrix(const nav_msgs::OccupancyGrid &metric_map)
{
  // Assume both representations to be row-major, but the matrix is like an image
  // having the upper left corner of the map in coordinates (0, 0),
  // while the occupancy map has (0, 0) in lower left corner
  cv::Mat mat(metric_map.info.height, metric_map.info.width, CV_8UC1, 255);
  int i = 0;
  for (int y = metric_map.info.height - 1; y >= 0; --y)
  {
    for (int x = 0; x < metric_map.info.width; ++x)
    {
      switch (metric_map.data[i])
      {
        case 0:  // Free space
          mat.at<uint8_t>(y, x) = 255;
          break;
        case 100:  // Occupied space
          mat.at<uint8_t>(y, x) = 0;
          break;
        case -1:  // Unknown space
          mat.at<uint8_t>(y, x) = 100;
          break;
        default:  // Means non-trinary map
          throw TopoException(str(format(
              "The map is NOT trinary (has value %d)! Only trinary maps are supported at this time!")
                                  % metric_map.data[i]));
      }
      ++i;
    }
  }

  return mat;
}


}  // namespace infobot_topo_mapping
