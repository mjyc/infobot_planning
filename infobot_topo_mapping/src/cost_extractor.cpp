#include "infobot_topo_mapping/cost_extractor.h"

#include <opencv2/imgproc/imgproc.hpp>

#include "evg-thin/evg-thin.hh"
#include "infobot_topo_mapping/visualizer.h"
#include "infobot_topo_mapping/topo_exception.h"


namespace infobot_topo_mapping
{

// -----------------------------------------------------------------------------
cv::Mat CostExtractor::getObstacleDistance(const cv::Mat &occupancy_mat,
                                           double map_resolution) const
{
  // Calculate distance transform
  cv::Mat obstacle_dist;
  // Returns 32FC1
  cv::distanceTransform(occupancy_mat, obstacle_dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  // Convert to meters
  obstacle_dist = obstacle_dist * map_resolution;

  return obstacle_dist;
}


// -----------------------------------------------------------------------------
cv::Mat CostExtractor::getObstacleCost(const cv::Mat &occupancy_mat,
                                       const cv::Mat &obstacle_dist) const
{
  if (obstacle_dist.type() != CV_32FC1)
    throw TopoException("Obstacle distance matrix must be of type CV_32FC1.");

  // From http://wiki.ros.org/costmap_2d/hydro/inflation
  // The cost function is computed as follows for all cells in the costmap
  // further than the inscribed radius distance and closer than the inflation
  // radius distance away from an actual obstacle:
  // exp(-1.0 * cost_scaling_factor * (distance_from_obstacle - inscribed_radius)) *
  // (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1), where
  // costmap_2d::INSCRIBED_INFLATED_OBSTACLE is currently 254. NOTE: since the
  // cost_scaling_factor is multiplied by a negative in the formula, increasing
  // the factor will decrease the resulting cost values.

  // Calculate the obstacle cost
  int count = obstacle_dist.total();
  const float* obstacle_dist_p = obstacle_dist.ptr<float>();
  cv::Mat obstacle_cost(obstacle_dist.size(), CV_32FC1);
  float* obstacle_cost_p = obstacle_cost.ptr<float>();

  for (int i = 0; i < count; ++i)
  {
    float dist = obstacle_dist_p[i];
    if ((dist < min_obstacle_dist_) || (occupancy_mat.data[i] != 255))
      obstacle_cost_p[i] = 1.0;
    else
      obstacle_cost_p[i] = exp(-1.0 * obstacle_cost_scaling_ * (dist - min_obstacle_dist_));
  }

  return obstacle_cost;
}


// -----------------------------------------------------------------------------
cv::Mat CostExtractor::getViewCost(const cv::Mat &obstacle_dist) const
{
  if (obstacle_dist.type() != CV_32FC1)
    throw TopoException("Obstacle distance matrix must be of type CV_32FC1.");

  // Calculate the cost
  int count = obstacle_dist.total();
  const float* obstacle_dist_p = obstacle_dist.ptr<float>();
  cv::Mat view_cost(obstacle_dist.size(), CV_32FC1);
  float* view_cost_p = view_cost.ptr<float>();

  for (int i = 0; i < count; ++i)
  {
    float dist = obstacle_dist_p[i];
    view_cost_p[i] = 1.0 - exp(-1.0 * fabs(dist - optimal_view_dist_) * view_cost_scaling_);
  }

  // FIX: Make this a param
  return view_cost * 1.0;
}


// -----------------------------------------------------------------------------
skeleton_ptr CostExtractor::getVoronoiGraph(const cv::Mat &occupancy_mat,
                                            double map_resolution,
                                            const cv::Point &seed) const
{
  // Convert to a grid for thinning
  // Matrices are row-major and data[0] represents the upper left corner of the map
  // The thinning library uses vector of columns, which makes it column-major
  column_type grid_col(occupancy_mat.rows, Unknown);
  grid_type grid(occupancy_mat.cols, grid_col);
  int i = 0;
  for (int y = 0; y < occupancy_mat.rows; ++y)
  {
    for (int x = 0; x < occupancy_mat.cols; ++x)
    {
      switch (occupancy_mat.data[i])
      {
        case 0:  // Occupied space
          grid[x][y] = Occupied;
          break;
        case 100:  // Unknown space
          grid[x][y] = Unknown;
          break;
        case 255:  // Free space
          grid[x][y] = Free;
          break;
      }
      ++i;
    }
  }

  // Perform thinning
  evg_thin thin(grid, min_voronoi_dist_ / map_resolution,
                100000, false, true, seed.x, seed.y);
  return thin.generate_skeleton_ptr();
}


// -----------------------------------------------------------------------------
cv::Mat CostExtractor::getCenterDistance(const cv::Mat &occupancy_mat,
                                         double map_resolution,
                                         const cv::Point &seed) const
{
  skeleton_ptr skel = getVoronoiGraph(occupancy_mat, map_resolution, seed);

  // Draw the skeleton on matrix
  cv::Mat skel_mat(occupancy_mat.size(), CV_8UC1, 255);
  for (int i=0; i < skel->size(); ++i)
    skel_mat.at<uint8_t>((*skel)[i].y, (*skel)[i].x) = 0;

  // Show the skeleton
  visualizer_.showVoronoi(3, "Skeleton", skel_mat, occupancy_mat, seed);

  // Calculate distance transform
  cv::Mat skel_dist;
  // Returns 32FC1
  cv::distanceTransform(skel_mat, skel_dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  // Convert to meters
  skel_dist = skel_dist * map_resolution;

  return skel_dist;
}


// -----------------------------------------------------------------------------
cv::Mat CostExtractor::getCenterCost(const cv::Mat &occupancy_mat,
                                     double map_resolution,
                                     const cv::Point &seed) const
{
  // Calculate distance to voronoi graph
  cv::Mat center_dist = getCenterDistance(occupancy_mat, map_resolution, seed);
  if (center_dist.type() != CV_32FC1)
    throw TopoException("Center distance matrix must be of type CV_32FC1.");

  // Visualize center distance
  visualizer_.showValueMap(3, "CenterDistance",
                           center_dist, occupancy_mat,
                           seed, cv::COLORMAP_HOT);

  // Calculate the center cost
  int count = center_dist.total();
  const float* center_dist_p = center_dist.ptr<float>();
  cv::Mat center_cost(center_dist.size(), CV_32FC1);
  float* center_cost_p = center_cost.ptr<float>();

  for (int i = 0; i < count; ++i)
  {
    float dist = center_dist_p[i];
    center_cost_p[i] = 1.0 - exp(-1.0 * dist * center_cost_scaling_);
  }

  return center_cost;
}


// -----------------------------------------------------------------------------
void CostExtractor::calculateAllCosts(const cv::Mat &occupancy_mat,
                                         double map_resolution,
                                         const cv::Point &seed)
{
  // Obstacle distance
  cv::Mat obstacle_dist = getObstacleDistance(occupancy_mat, map_resolution);
  visualizer_.showValueMap(3, "ObstacleDistance",
                           obstacle_dist, occupancy_mat,
                           seed, cv::COLORMAP_HOT);

  // Obstacle cost
  obstacle_cost = getObstacleCost(occupancy_mat, obstacle_dist);
  visualizer_.showValueMap(2, "ObstacleCost", obstacle_cost, occupancy_mat, seed);

  // Optimal view cost
  view_cost = getViewCost(obstacle_dist);
  visualizer_.showValueMap(2, "ViewCost", view_cost, occupancy_mat, seed);

  // Center cost
  center_cost = getCenterCost(occupancy_mat, map_resolution, seed);
  visualizer_.showValueMap(2, "CenterCost", center_cost, occupancy_mat, seed);
}


// -----------------------------------------------------------------------------
cv::Mat CostExtractor::getCostMap(const cv::Mat &occupancy_mat,
                                  double map_resolution,
                                  const cv::Point &seed)
{
  // Calculate all cost maps
  calculateAllCosts(occupancy_mat, map_resolution, seed);

  // Threshold the cost to always make sure that we keep the cost as max
  // when it's 1.0 in obstacle_cost.
  cv::Mat obstacle_cost_thr;
  cv::threshold(obstacle_cost, obstacle_cost_thr, 0.999, 1.0, cv::THRESH_BINARY);

  // Combined cost
  cv::Mat cost;
  double weight_sum = obstacle_cost_weight_ + view_cost_weight_ + center_cost_weight_;

  // Combined cost
  cost =  obstacle_cost * (obstacle_cost_weight_ / weight_sum) +
          view_cost * (view_cost_weight_ / weight_sum) +
          center_cost * (center_cost_weight_ / weight_sum);
  cost = cv::max(cost, obstacle_cost_thr);  // NOLINT(build/include_what_you_use)
  visualizer_.showValueMap(1, "FinalCostMap", cost, occupancy_mat, seed);

  return cost;
}

// -----------------------------------------------------------------------------
cv::Mat CostExtractor::getProbabilityMap(const cv::Mat &occupancy_mat,
                                         double map_resolution,
                                         const cv::Point &seed)
{
  // Calculate all cost maps
  calculateAllCosts(occupancy_mat, map_resolution, seed);

  //  Get combined cost as probabilities
  cv::Mat prob;
  prob =  (1.0 - obstacle_cost).mul(
      (1.0 - center_cost) +
      (1.0 - view_cost) -
      (1.0 - center_cost).mul(1.0 - view_cost));
  visualizer_.showValueMap(1, "FinalProbabilityMap", prob, occupancy_mat, seed);

  return prob;
}

}  // namespace infobot_topo_mapping
