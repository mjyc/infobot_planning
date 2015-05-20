#include "infobot_topo_mapping/place_sampler.h"

#include <boost/random/bernoulli_distribution.hpp>
#include <algorithm>
#include <vector>
#include <ctime>
#include <iostream>

#include "infobot_topo_mapping/visualizer.h"


namespace infobot_topo_mapping
{

using std::vector;
using boost::random::bernoulli_distribution;
using boost::random::mt19937;

// -----------------------------------------------------------------------------
cv::Mat PlaceSampler::getInitialPlaceMapSample(const cv::Mat &goodness_map,
                                               const vector<cv::Point> &seed_places,
                                               double map_resolution) const
{
  cv::Mat map = cv::Mat::zeros(goodness_map.size(), CV_8UC1);

  // FIX: consider using the grid
  int optimal_dist_cell = optimal_place_dist_ / map_resolution;
  int width = goodness_map.size().width;
  int height = goodness_map.size().height;

  // Grid of places optimally spaced
  // for (int y = optimal_dist_cell; y < height; y += optimal_dist_cell)
  // {
  //   for (int x = optimal_dist_cell; x < width; x += optimal_dist_cell)
  //   {
  //     if (goodness_map.at<float>(y, x) > 0.0)
  //       map.at<uint8_t>(y, x) = 255;
  //   }
  // }

  // Add seed places
  for (int i = 0; i < seed_places.size(); ++i)
  {
    map.at<uint8_t>(seed_places[i].y, seed_places[i].x) = 255;
  }

  return map;
}


// -----------------------------------------------------------------------------
float PlaceSampler::getPlaceConditional(int hyp_x, int hyp_y,
                                        int radius,
                                        const cv::Mat &goodness_map,
                                        // const cv::Mat &rel_kernel,
                                        const cv::Mat &place_map) const
{
  int start_x = std::max(hyp_x - radius, 0);
  int start_y = std::max(hyp_y - radius, 0);
  int end_x = std::min(hyp_x + radius, place_map.size().width - 1);
  int end_y = std::min(hyp_y + radius, place_map.size().height - 1);

  // Find the distance to the nearest place
  // TODO: This should be optimized by e.g.KD-tree
  float dist = 100000000.0;
  for (int y = start_y; y <= end_y; ++y)
    for (int x = start_x; x <= end_x; ++x)
    {
      if ((place_map.at<uint8_t>(y, x) > 0) &&
          ((y != hyp_y) || (x != hyp_x)))
      {
        float d =  sqrt((hyp_x - x) * (hyp_x - x) +
                        (hyp_y - y) * (hyp_y - y));
        if (d < dist)
        {
          dist = d;
        }
      }
    }

  if (dist < radius)
  {
    return 0.0;
  }
  else
  {
    // Multiply by the goodness probability
    float g = goodness_map.at<float>(hyp_y, hyp_x);

    // FIX: Theses below are constants!!
    return g;  // / (g + 0.1 * 0.1);
  }
}


// -----------------------------------------------------------------------------
float PlaceSampler::getPlaceFactors(int hyp_x, int hyp_y,
                                    int radius,
                                    const cv::Mat &goodness_map,
                                    const cv::Mat &place_map) const
{
  // Return a small constant for pi = 0
  if (place_map.at<uint8_t>(hyp_y, hyp_x) == 0)
  {
    return 0.1 * 0.1;  // FIX: constants as above
  }
  // Else:

  int start_x = std::max(hyp_x - radius, 0);
  int start_y = std::max(hyp_y - radius, 0);
  int end_x = std::min(hyp_x + radius, place_map.size().width - 1);
  int end_y = std::min(hyp_y + radius, place_map.size().height - 1);

  // Find the distance to the nearest place
  // TODO: This should be optimized by e.g.KD-tree
  float dist = 100000000.0;
  for (int y = start_y; y <= end_y; ++y)
    for (int x = start_x; x <= end_x; ++x)
    {
      if ((place_map.at<uint8_t>(y, x) > 0) &&
          ((y != hyp_y) || (x != hyp_x)))
      {
        float d =  sqrt((hyp_x - x) * (hyp_x - x) +
                        (hyp_y - y) * (hyp_y - y));
        if (d < dist)
        {
          dist = d;
        }
      }
    }

  if (dist < radius)
  {
    return 0.0;
    std::cout<<"Place conflict!"<<std::endl;
  }
  else
  {
    // Multiply by the goodness probability
    float g = goodness_map.at<float>(hyp_y, hyp_x);

    if (g == 0.0)
    {
      std::cout<<"Goodness 0"<<hyp_x << " " << hyp_y << std::endl;
    }

    return g;
  }
}


// -----------------------------------------------------------------------------
cv::Mat PlaceSampler::performGibbsIteration(cv::Mat sample,
                                            const cv::Mat &goodness_map,
                                            const cv::Mat &occupancy_map,
                                            const vector<cv::Point> &seed_places,
                                            int radius,
                                           // const cv::Mat &rel_kernel,
                                            boost::random::mt19937 *gen) const
{
  // FIX: this is just for testing
  // cv::Mat place_cond = cv::Mat::zeros(sample.size(), CV_32FC1);

  for (int y = 0; y < sample.rows; ++y)
  {
    for (int x = 0; x < sample.cols; ++x)
    {
      // Operate only on non - seed places
      if (!inPoints(seed_places, x, y))
      {
        float cond = getPlaceConditional(x, y,
                                         radius,
                                         goodness_map,
                                         // rel_kernel,
                                         sample);
        // FIX: Just for testing
        // place_cond.at<float>(y, x) =  cond;

        // Sample new value of p_i
        bernoulli_distribution<float> dist(cond);
        if (dist(*gen))
          sample.at<uint8_t>(y, x) = 255;
        else
          sample.at<uint8_t>(y, x) = 0;
      }
    }
  }
  // visualizer_.showValueMap(2, "PlaceConditionalMap",
  //                          place_cond, occupancy_map,
  //                          seed_places);
  visualizer_.showMatrix(1, "PlaceMapSample", sample);

  return sample;
}


// -----------------------------------------------------------------------------
double PlaceSampler::getPlaceMapProbability(const cv::Mat &place_map,
                                           const cv::Mat &goodness_map,
                                           const cv::Mat &occupancy_map,
                                           const vector<cv::Point> &seed_places,
                                           int radius) const
{
  double log_prob = 0.0;
  // cv::Mat map =  cv::Mat::zeros(place_map.size(),  CV_32FC1);

  for (int y = 0; y < place_map.rows; ++y)
  {
    for (int x = 0; x < place_map.cols; ++x)
    {
      float prob = getPlaceFactors(x, y,
                                   radius,
                                   goodness_map,
                                   place_map);

      // map.at<float>(y, x) = prob;
      log_prob += log(prob);
    }
  }

  // visualizer_.showValueMap(2, "PlaceFactorMap",
  //                          map, occupancy_map,
  //                          seed_places);

  return log_prob;
}



// -----------------------------------------------------------------------------
cv::Mat PlaceSampler::samplePlaces(const cv::Mat &goodness_map,
                                   const cv::Mat &occupancy_map,
                                   const vector<cv::Point> &seed_places,
                                   double map_resolution) const
{
  // Check if we have at least one seed place
  if (seed_places.empty())
    throw("At lease one seed place is required for place sampling!");

  int radius = round(optimal_place_dist_ / map_resolution);

  //  Get relation kernel
  // cv::Mat rel_kernel =  getRelationKernel(map_resolution);
  // visualizer_.showMatrix(1, "PlaceRelationKernel", rel_kernel);

  // Initial place map sample
  cv::Mat place_map_sample = getInitialPlaceMapSample(goodness_map,
                                                      seed_places,
                                                      map_resolution);
  visualizer_.showMatrix(3, "InitialPlaceMapSample", place_map_sample);

  // Perform gibbs sampling
  mt19937 gen(std::time(0));
  // cv::Mat posterior = cv::Mat::zeros(place_map_sample.size(), CV_32FC1);
  double max_prob =- 100000000.0;
  cv::Mat max_sample;

  for (int i = 0; (i < gibbs_max_iter_) && (visualizer_.processEvents()); ++i)
  {
    place_map_sample = performGibbsIteration(place_map_sample,
                                             goodness_map,
                                             occupancy_map,
                                             seed_places,
                                             radius,
                                             // rel_kernel,
                                             &gen);

    std::cout << "Iteration: " << i << std::endl;

    // Evaluate the sample
    double prob =  getPlaceMapProbability(place_map_sample, goodness_map, occupancy_map,
                                         seed_places, radius);
    if (prob > max_prob)
    {
      max_prob = prob;
      max_sample = place_map_sample.clone();
      visualizer_.showMatrix(1, "MaxPlaceMapSample", max_sample);
      // visualizer_.waitForKey();
    }

    std::cout << "Log prob: " << prob << " max: " << max_prob << std::endl;
  }

  return max_sample;
}



}  // namespace infobot_topo_mapping





// // -----------------------------------------------------------------------------
// float PlaceSampler::getRelationConditional2(int hyp_x, int hyp_y,
//                                             const std::vector<cv::Point> &seed_places,
//                                             const cv::Mat &rel_kernel,
//                                             const cv::Mat &place_map) const
// {
//   // Be careful about seed places which should be considered
//   // even if they are before x, y!!
//   // since we calculate: P(r_i | p_i+1, ..., p_N, seed_places)
//   // where seed places are given!

//   int radius = (rel_kernel.size().width - 1) / 2;
//   int start_x = std::max(hyp_x - radius, 0);
//   int start_y = std::max(hyp_y - radius, 0);
//   int end_x = std::min(hyp_x + radius, place_map.size().width - 1);
//   int end_y = std::min(hyp_y + radius, place_map.size().height - 1);

//   // Find the distance to the nearest place
//   float dist = 100000000.0;
//   int dist_x = -1;
//   int dist_y = -1;

//   // Start with seed places since they are always considered
//   for (int i = 0; i < seed_places.size(); ++i)
//   {
//     int x = seed_places[i].x;
//     int y = seed_places[i].y;
//     // If x, y within the boundaries of the kernel
//     if ((x >= start_x) && (x <= end_x) &&
//         (y >= start_y) && (y <= end_y))
//     {
//       float d =  sqrt((hyp_x - x) * (hyp_x - x) +
//                       (hyp_y - y) * (hyp_y - y));
//       if (d < dist)
//       {
//         dist = d;
//         dist_x = x;
//         dist_y = y;
//       }
//     }
//   }

//   // TODO: This should be optimized by e.g.KD-tree
//   for (int y = start_y; y <= end_y; ++y)
//     for (int x = start_x; x <= end_x; ++x)
//     {
//       // Calculate only for places existing in the given map
//       // but exclude everything until hyp_i + 1
//       if ((place_map.at<uint8_t>(y, x) > 0) &&
//           ((y > hyp_y) ||
//            ((y == hyp_y) && (x > hyp_x))))
//       {
//         float d =  sqrt((hyp_x - x) * (hyp_x - x) +
//                         (hyp_y - y) * (hyp_y - y));
//         if (d < dist)
//         {
//           dist = d;
//           dist_x = x;
//           dist_y = y;
//         }
//       }
//     }

//   // Assume that all values are zero outside the kernel since the places are
//   // too far. Assume the seed place was somewhere there, but it's too far
//   // so we set the conditional to 0
//   if (dist_x < 0)
//     return 0.0;
//   else
//     return rel_kernel.at<float>(
//         radius + (dist_y - hyp_y), radius + (dist_x - hyp_x));
// }



// -----------------------------------------------------------------------------
// cv::Mat PlaceSampler::getRelationKernel(double map_resolution) const
// {
//   float dist =  optimal_place_dist_ / map_resolution;
//   float sigma =  place_dist_sigma_ / map_resolution;
//   float var2 = 2 * sigma * sigma;

//   // Estimate size of the kernel.
//   // We use 2 * (3 * sigma + optimal_place_dist) + 1
//   int radius =  round(dist + 3.0 * sigma);
//   int size =  2 * radius + 1;

//   cv::Mat kernel =  cv::Mat(size, size, CV_32FC1);
//   for (int y = 0; y < size; ++y)
//     for (int x = 0; x < size; ++x)
//     {
//       float d =  sqrt((radius - x) * (radius - x) +
//                       (radius - y) * (radius - y));
//       kernel.at<float>(y, x) = exp(- (d - dist) * (d - dist) / var2);
//       // kernel.at<float>(y, x) = exp(- fabs(d - dist) / var2);
//       // kernel.at<float>(y, x) = (fabs(d - dist) < 2) ? 1.0 : 0.0;
//       // kernel.at<float>(y, x) = (fabs(d - dist) < 2) ? (exp(- (d - dist) * (d - dist) / var2)) : 0.0;

//     }

//   return kernel;
// }



// // -----------------------------------------------------------------------------
// float PlaceSampler::getPlaceConditional2(int hyp_x, int hyp_y,
//                                          const std::vector<cv::Point> &seed_places,
//                                          const cv::Mat &goodness_map,
//                                          const cv::Mat &rel_kernel,
//                                          const cv::Mat &place_map) const
// {
//   float cond = getRelationConditional2(hyp_x, hyp_y,
//                                        seed_places,
//                                        rel_kernel,
//                                        place_map);

//   // Multiply by the goodness probability
//   cond *= goodness_map.at<float>(hyp_y, hyp_x);

//   return cond;
// }



// -----------------------------------------------------------------------------
// float PlaceSampler::getPlaceConditional(int hyp_x, int hyp_y,
//                                         const cv::Mat &goodness_map,
//                                         const cv::Mat &rel_kernel,
//                                         const cv::Mat &place_map) const
// {
//   float cond = getRelationConditional(hyp_x, hyp_y,
//                                       rel_kernel,
//                                       place_map);

//   // Multiply by the goodness probability
//   cond *= goodness_map.at<float>(hyp_y, hyp_x);

//   return cond;
// }





// // -----------------------------------------------------------------------------
// float PlaceSampler::getRelationConditional(int hyp_x, int hyp_y,
//                                            const cv::Mat &rel_kernel,
//                                            const cv::Mat &place_map) const
// {
//   int radius = (rel_kernel.size().width - 1) / 2;
//   int start_x = std::max(hyp_x - radius, 0);
//   int start_y = std::max(hyp_y - radius, 0);
//   int end_x = std::min(hyp_x + radius, place_map.size().width - 1);
//   int end_y = std::min(hyp_y + radius, place_map.size().height - 1);

//   // Find the distance to the nearest place
//   // TODO: This should be optimized by e.g.KD-tree
//   float dist = 100000000.0;
//   int dist_x = -1;
//   int dist_y = -1;
//   for (int y = start_y; y <= end_y; ++y)
//     for (int x = start_x; x <= end_x; ++x)
//     {
//       // Calculate only for places existing in the given map
//       // but exclude the hyp_x, hyp_y place itself since the conditional
//       // is over all other places
//       if ((place_map.at<uint8_t>(y, x) > 0) &&
//           ((y != hyp_y) || (x != hyp_x)))
//       {
//         float d =  sqrt((hyp_x - x) * (hyp_x - x) +
//                         (hyp_y - y) * (hyp_y - y));
//         if (d < dist)
//         {
//           dist = d;
//           dist_x = x;
//           dist_y = y;
//         }
//       }
//     }

//   // NOT USED!
//   // Get the values of the kernel placed at hyp_x, hyp_y for the existing places
//   // float prob = 1.0;
//   // bool place_found = false;
//   // for (int y = start_y; y <= end_y; ++y)
//   //   for (int x = start_x; x <= end_x; ++x)
//   //   {
//   //     // Calculate only for places existing in the given map
//   //     // but exclude the hyp_x, hyp_y place itself since the conditional
//   //     // is over all other places
//   //     if ((place_map.at<uint8_t>(y, x) > 0) &&
//   //         ((y != hyp_y) || (x != hyp_x)))
//   //     {
//   //       prob *= rel_kernel.at<float>(radius + (y - hyp_y), radius + (x - hyp_x));
//   //       place_found = true;
//   //     }
//   //   }
//   // NOT USED!

//   // Assume that all values are zero outside the kernel since the places are
//   // too far. Assume the seed place was somewhere there, but it's too far
//   // so we set the conditional to 0
//   if (dist_x < 0)
//     return 0.0;
//   else
//     return rel_kernel.at<float>(
//         radius + (dist_y - hyp_y), radius + (dist_x - hyp_x));
// }




// // -----------------------------------------------------------------------------
// cv::Mat PlaceSampler::getRelationConditionalMap(const cv::Mat &place_map,
//                                                 const cv::Mat &rel_kernel) const
// {
//   cv::Mat map = cv::Mat(place_map.size(), CV_32FC1);
//   for (int y = 0; y < place_map.rows; ++y)
//     for (int x = 0; x < place_map.cols; ++x)
//     {
//       map.at<float>(y, x) = getRelationConditional(x, y,
//                                                    rel_kernel,
//                                                    place_map);
//     }

//   return map;
// }



// -----------------------------------------------------------------------------
// float PlaceSampler::getPlaceMapProbability(const cv::Mat &place_map,
//                                            const cv::Mat &goodness_map,
//                                            const cv::Mat &occupancy_map,
//                                            const vector<cv::Point> &seed_places,
//                                            const cv::Mat &rel_kernel) const
// {
//   float log_prob = 0.0;
//   cv::Mat map =  cv::Mat::zeros(place_map.size(),  CV_32FC1);

//   for (int y = 0; y < place_map.rows; ++y)
//   {
//     for (int x = 0; x < place_map.cols; ++x)
//     {
//       // Operate only on non - seed places
//       // and on places that exist in the place map
//       if (!inPoints(seed_places, x, y))
//       {
//         float prob =  getRelationConditional(hyp_x, hyp_y,
//                                              rel_kernel,
//                                              place_map);

//         // float prob =  getPlaceConditional(x, y,
//         //                                   // seed_places,
//         //                                   goodness_map,
//         //                                   rel_kernel,
//         //                                   place_map);
//         if (place_map.at<uint8_t>(y, x) == 0)
//         {
//           prob = prob *  0.5;
//         }
//         else
//         {
//           prob =
//           std::cout << "P: " << prob << std::endl;
//         }

//         map.at<float>(y, x) = prob;
//         log_prob += log(prob);
//       }
//     }
//   }

//   visualizer_.showValueMap(1, "PlacePartialConditionalMap",
//                            map, occupancy_map,
//                            seed_places);

//   return log_prob;
// }
