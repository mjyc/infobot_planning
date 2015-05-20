#ifndef _INFOBOT_TOPO_MAPPING__PLACE_SAMPLER_
#define _INFOBOT_TOPO_MAPPING__PLACE_SAMPLER_

#include <boost/random/mersenne_twister.hpp>
#include <opencv2/core/core.hpp>
#include <vector>


namespace infobot_topo_mapping
{

class Visualizer;

/**
 * Samples a grid of place centers given goodness / cost.
 */
class PlaceSampler
{
 public:
  explicit PlaceSampler(const Visualizer &visualizer)
      : visualizer_(visualizer),
        optimal_place_dist_(1.0),
        place_dist_sigma_(0.1),
        gibbs_max_iter_(100),
        gibbs_burnin_(20)
  {}

  /** Sets the algorithm parameters. */
  void setParams(double optimal_place_dist,
                 double place_dist_sigma)
  {
    optimal_place_dist_ = optimal_place_dist;
    place_dist_sigma_ = place_dist_sigma;
  }

  /** Samples a grid of place centers for the given
      goodness probability map. */
  cv::Mat samplePlaces(const cv::Mat &goodness_map,
                       const cv::Mat &occupancy_map,
                       const std::vector<cv::Point> &seed_places,
                       double map_resolution) const;


 private:
  /** Visualizer used for debugging. */
  const Visualizer &visualizer_;

  // Params
  double optimal_place_dist_;
  double place_dist_sigma_;
  int gibbs_max_iter_;
  int gibbs_burnin_;

 private:
  /**
     Provide initial sample of p(P|M) for Gibbs
   */
  cv::Mat getInitialPlaceMapSample(const cv::Mat & goodness_map,
                                   const std::vector<cv::Point>& seed_places,
                                   double map_resolution) const;

  /**
     Calculate a matrix containing probability values for place relations
     p(r_0,0 | p_x,y) assuming that point 0,0 is in the middle of the matrix.
     We assume p(r) = 0 outside that kernel.
   */
  // cv::Mat getRelationKernel(double map_resolution) const;

  /**
     Provide estimate of the conditional probability of being in good
     relation given existence of other places in other locations.
     P(r_i | p_1, .., p_i-1, p_i+1, ..., p_N, seed_places)
     Assume that the seed places are already set in the given place map
     and at least one is required!
     Params:
     - hyp_x, hyp_y -  cell for which the conditional is calculated
     - rel_kernel - probability values p(r_0,0 | p_x,y).
                    We assume = 0 outside that kernel.
     - place_map - Place map from which we take the p_!i values
   */
  float getRelationConditional(int hyp_x, int hyp_y,
                               const cv::Mat &rel_kernel,
                               const cv::Mat &place_map) const;

  float getPlaceFactors(int hyp_x, int hyp_y,
                        int radius,
                        const cv::Mat &goodness_map,
                        const cv::Mat &place_map) const;


  /**
     As above, but calculate only:
     P(r_i | p_i+1, ..., p_N, seed_places)
   */
  // float getRelationConditional2(int hyp_x, int hyp_y,
  //                               const std::vector<cv::Point> &seed_places,
  //                               const cv::Mat &rel_kernel,
  //                               const cv::Mat &place_map) const;


  /**
     Calculate the conditional
     P(r_i | p_1, .., p_i-1, p_i+1, ..., p_N, seed_places)
     for every i.
     Assume that the seed places are already set in the given place map
     and at least one is required! Please note that the value for the
     seed places themselves does not make any sense and should never be
     computed!
  */
  // cv::Mat getRelationConditionalMap(const cv::Mat &place_map,
  //                                   const cv::Mat &rel_kernel) const;

  /**
     Provide estimate of the conditional probability of place
     existence given the existence of other places in other locations.
     P(p_i | p_1, .., p_i-1, p_i+1, ..., p_N, seed_places, M)
     Assume that the seed places are already set in the given place map
     and at least one is required!
     Params:
     - hyp_x, hyp_y -  cell for which the conditional is calculated
     - goodness_map - probability values of a location being good
     - rel_kernel - probability values p(r_0,0 | p_x,y).
                    We assume = 0 outside that kernel.
     - place_map - Place map from which we take the p_!i values
     - cond_start_x,  cond_start_y - calculate:
       P(p_i | p_cond_start, .., p_i-1, p_i+1, ..., p_N, seed_places, M)
   */
  float getPlaceConditional(int hyp_x, int hyp_y,
                            int radius,
                            const cv::Mat &goodness_map,
                            // const cv::Mat &rel_kernel,
                            const cv::Mat &place_map) const;

  /**
     As above, but calculate only:
     P(p_i | p_i+1, ..., p_N, seed_places, M)
  */
  // float getPlaceConditional2(int hyp_x, int hyp_y,
  //                            const std::vector<cv::Point> &seed_places,
  //                            const cv::Mat &goodness_map,
  //                            const cv::Mat &rel_kernel,
  //                            const cv::Mat &place_map) const;


  bool inPoints(const std::vector<cv::Point> &seed_places, int x, int y) const
  {
    for (int i = 0; i < seed_places.size(); ++i)
      if ((seed_places[i].x == x) && (seed_places[i].y == y))
        return true;

    return false;
  }


  cv::Mat performGibbsIteration(cv::Mat sample,
                                const cv::Mat &goodness_map,
                                const cv::Mat &occupancy_map,
                                const std::vector<cv::Point> &seed_places,
                                int radius,
                                // const cv::Mat &rel_kernel,
                                boost::random::mt19937 *gen) const;

  /**
     Calculate the probability P(P | s, M)
   */
  double getPlaceMapProbability(const cv::Mat & place_map,
                               const cv::Mat &goodness_map,
                               const cv::Mat &occupancy_map,
                               const std::vector<cv::Point> &seed_places,
                               int radius) const;

  /**
     Perform complete Gibbs sampling.
   */
  cv::Mat performGibbsSampling(const cv::Mat &goodness_map,
                               const std::vector<cv::Point> &seed_places,
                               const cv::Mat &rel_kernel) const;
};

}  // namespace infobot_topo_mapping

#endif
