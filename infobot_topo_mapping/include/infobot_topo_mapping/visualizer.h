#ifndef _INFOBOT_TOPO_MAPPING__VISUALIZER_
#define _INFOBOT_TOPO_MAPPING__VISUALIZER_

#include <opencv2/contrib/contrib.hpp>  // for colormap definitions
#include <string>
#include <vector>


namespace infobot_topo_mapping
{

/**
 * Internal visualization for testing.
 */
class Visualizer
{
 public:
  Visualizer() :
      visualization_level_(0)
  {}

  void setParams(int visualization_level)
  {
    visualization_level_ = visualization_level;
  }

  void showMatrix(int level,
                  const std::string &win_name,
                  const cv::Mat &mat) const;

  void showVoronoi(int level,
                   const std::string &win_name,
                   const cv::Mat &skel,
                   const cv::Mat &occupancy,
                   const cv::Point &seed) const;

  void showValueMap(int level,
                    const std::string &win_name,
                    const cv::Mat &values,
                    const cv::Mat &occupancy,
                    const std::vector<cv::Point> &seed_places,
                    const int color_map = cv::COLORMAP_JET) const;

  void showValueMap(int level,
                    const std::string &win_name,
                    const cv::Mat &values,
                    const cv::Mat &occupancy,
                    const cv::Point &seed_place,
                    const int color_map = cv::COLORMAP_JET) const
  {
    std::vector<cv::Point> seed_places;
    seed_places.push_back(seed_place);
    showValueMap(level,  win_name, values, occupancy, seed_places, color_map);
  }

  bool processEvents() const;

  bool waitForKey() const;


 private:
  int visualization_level_;
};

}  // namespace infobot_topo_mapping

#endif
