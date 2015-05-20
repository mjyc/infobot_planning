#ifndef _INFOBOT_TOPO_MAPPING__COST_EXTRACTOR_
#define _INFOBOT_TOPO_MAPPING__COST_EXTRACTOR_

#include <opencv2/core/core.hpp>
#include "evg-thin/datatypes.hh"


namespace infobot_topo_mapping
{

class Visualizer;

/**
 * Extracts a place position cost map from a metric map.
 */
class CostExtractor
{
 public:
  explicit CostExtractor(const Visualizer &visualizer)
      : visualizer_(visualizer),
        min_obstacle_dist_(0.0),
        optimal_view_dist_(0.0),
        min_voronoi_dist_(0.0),
        obstacle_cost_scaling_(0.0),
        view_cost_scaling_(0.0),
        center_cost_scaling_(0.0),
        obstacle_cost_weight_(0.0),
        view_cost_weight_(0.0),
        center_cost_weight_(0.0)
  {}

  /** Sets the algorithm parameters. */
  void setParams(double min_obstacle_dist,
                 double optimal_view_dist,
                 double min_voronoi_dist,
                 double obstacle_cost_scaling,
                 double view_cost_scaling,
                 double center_cost_scaling,
                 double obstacle_cost_weight,
                 double view_cost_weight,
                 double center_cost_weight)
  {
    min_obstacle_dist_     = min_obstacle_dist;
    optimal_view_dist_     = optimal_view_dist;
    min_voronoi_dist_      = min_voronoi_dist;
    obstacle_cost_scaling_ = obstacle_cost_scaling;
    view_cost_scaling_     = view_cost_scaling;
    center_cost_scaling_   = center_cost_scaling;
    obstacle_cost_weight_  = obstacle_cost_weight;
    view_cost_weight_      = view_cost_weight;
    center_cost_weight_    = center_cost_weight;
  }

  /** Calculates the final cost map. */
  cv::Mat getCostMap(const cv::Mat &occupancy_mat,
                     double map_resolution,
                     const cv::Point &seed);

  /** Calculates the final cost map as a probability map. */
  cv::Mat getProbabilityMap(const cv::Mat &occupancy_mat,
                            double map_resolution,
                            const cv::Point &seed);

 private:
  /** Visualizer used for debugging. */
  const Visualizer &visualizer_;

  // Params
  double min_obstacle_dist_;
  double optimal_view_dist_;
  double min_voronoi_dist_;
  double obstacle_cost_scaling_;
  double view_cost_scaling_;
  double center_cost_scaling_;
  double obstacle_cost_weight_;
  double view_cost_weight_;
  double center_cost_weight_;

  // Independent cost maps
  cv::Mat obstacle_cost;
  cv::Mat view_cost;
  cv::Mat center_cost;


 private:
  /** Calculates an image where each pixel is the distance to closest obstacle. */
  cv::Mat getObstacleDistance(const cv::Mat &occupancy_mat,
                                    double map_resolution) const;

  /** Calculates the obstacle cost related to the ability of robot to navigate safely to the place. */
  cv::Mat getObstacleCost(const cv::Mat &occupancy_mat,
                          const cv::Mat &obstacle_dist) const;

  /** Calculates the cost promoting optimal view locations for obstacles. */
  cv::Mat getViewCost(const cv::Mat &obstacle_dist) const;

  /** Calculates the voronoi graph for the given occupancy grid map. */
  skeleton_ptr getVoronoiGraph(const cv::Mat &occupancy_mat,
                               double map_resolution,
                               const cv::Point &seed) const;

  /** Calculates an image where each pixel is the distance to the closest voronoi graph pixel. */
  cv::Mat getCenterDistance(const cv::Mat &occupancy_mat,
                            double map_resolution,
                            const cv::Point &seed) const;

  /** Calculates the cost promoting centrally located places in open spaces. */
  cv::Mat getCenterCost(const cv::Mat &occupancy_mat,
                        double map_resolution,
                        const cv::Point &seed)  const;

  /** Calculates all the cost maps and stores them in the member variables. */
  void calculateAllCosts(const cv::Mat &occupancy_mat,
                         double map_resolution,
                         const cv::Point &seed);
};


}  // namespace infobot_topo_mapping


#endif
