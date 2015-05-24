#ifndef _INFOBOT_TOPO_MAPPING__TOPO_EXTRACTOR_
#define _INFOBOT_TOPO_MAPPING__TOPO_EXTRACTOR_

#include <nav_msgs/OccupancyGrid.h>
#include <infobot_topo_msgs/TopologicalMap.h>
#include <infobot_topo_msgs/MetricMapTransform.h>
#include <tf2/LinearMath/Transform.h>
#include <opencv2/core/core.hpp>

#include "infobot_topo_mapping/cost_extractor.h"
#include "infobot_topo_mapping/place_sampler.h"


namespace infobot_topo_mapping
{

class Visualizer;

/**
 * Extracts topology from metric maps.
 */
class TopoExtractor
{
 public:
  explicit TopoExtractor(const infobot_topo_msgs::TopologicalMap::Ptr &topo_map,
                         const Visualizer &visualizer)
      : topo_map_(topo_map),
        visualizer_(visualizer),
        cost_extractor_(visualizer),
        place_sampler_(visualizer)
  {}

  /** Sets the algorithm parameters. */
  void setCostExtractorParams(double min_obstacle_dist,
                 double optimal_view_dist,
                 double min_voronoi_dist,
                 double obstacle_cost_scaling,
                 double view_cost_scaling,
                 double center_cost_scaling,
                 double obstacle_cost_weight,
                 double view_cost_weight,
                 double center_cost_weight)
  {
    cost_extractor_.setParams(min_obstacle_dist,
                              optimal_view_dist,
                              min_voronoi_dist,
                              obstacle_cost_scaling,
                              view_cost_scaling,
                              center_cost_scaling,
                              obstacle_cost_weight,
                              view_cost_weight,
                              center_cost_weight);
  }

  void setPlaceSamplerParams(int min_obstacle_dist,
                             int optimal_view_dist)
  {
    ROS_WARN("DOES NOT CHECK INPUT PARAMETERS!! Make sure to use VALID INPUTS!!")
    place_sampler_.setParams(min_obstacle_dist,
                             optimal_view_dist);
  }

  /** Sets the current topological map. */
  void setTopoMap(const infobot_topo_msgs::TopologicalMap::Ptr &topo_map)
  {
    topo_map_ = topo_map;
  }

  /** Gets the current topo map. */
  infobot_topo_msgs::TopologicalMap::ConstPtr topoMap()
  {
    return topo_map_;
  }

  /** Process the given metric map.
      Assume that we are using seed view, so we create a new topo map.**/
  void processMetricMap(const nav_msgs::OccupancyGrid &metric_map,
                        const geometry_msgs::Pose2D &seed_view);

  /** Process the given metric map.
      Assume that we are using relations to other maps, so we update
      the existing topo map with new places. **/
  void processMetricMap(const nav_msgs::OccupancyGrid &metric_map,
                        const std::vector<infobot_topo_msgs::MetricMapTransform> &map_relations);

 private:
  /** Current topo map. */
  infobot_topo_msgs::TopologicalMap::Ptr topo_map_;

  /** Visualizer used for debugging. */
  const Visualizer &visualizer_;

  /** Cost map extractor. */
  CostExtractor cost_extractor_;

  /** Place sampler. */
  PlaceSampler place_sampler_;

  bool inPoints(const std::vector<cv::Point> &seed_places, int x, int y) const
  {
    for (int i = 0; i < seed_places.size(); ++i)
      if ((seed_places[i].x == x) && (seed_places[i].y == y))
        return true;

    return false;
  }



 private:
  /** Removes all places from the map. */
  void clearTopoMap();

  /** Convert the given map pose to matrix (x, y) cell coordinates */
  cv::Point getMatPoint(const nav_msgs::OccupancyGrid &metric_map,
                        const tf2::Transform &origin_transform,
                        const geometry_msgs::Pose2D &pose);

  /** Convert the given matrix (x, y) cell coordinates to map pose without theta.*/
  geometry_msgs::Pose2D getMapPose(const nav_msgs::OccupancyGrid &metric_map,
                                   const tf2::Transform &origin_transform,
                                   int x, int y, double angle);


  /** Converts the occupancy grid to opencv matrix. */
  cv::Mat getOccupancyMatrix(const nav_msgs::OccupancyGrid &metric_map);
};

}  // namespace infobot_topo_mapping

#endif
