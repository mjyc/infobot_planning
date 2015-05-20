#include "infobot_topo_mapping/visualizer.h"

#include <stdint.h>
#include <boost/format.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace infobot_topo_mapping
{
using boost::format;
using boost::str;

// -----------------------------------------------------------------------------
bool Visualizer::processEvents() const
{
  int key = 0;

  if (visualization_level_ > 0)
  {
    key = cv::waitKey(30);
  }

  return ros::ok() && (key != 27);;
}


// -----------------------------------------------------------------------------
bool Visualizer::waitForKey() const
{
  int key = 0;

  if (visualization_level_ > 0)
  {
    key = cv::waitKey();
  }

  return ros::ok() && (key != 27);
}


// -----------------------------------------------------------------------------
void Visualizer::showMatrix(int level, const std::string &win_name,
                            const cv::Mat &mat) const
{
  if (visualization_level_ >= level)
  {
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

    if ((mat.depth() == CV_32F) || (mat.depth() == CV_64F))
    {
      cv::Mat mat_norm;
      cv::normalize(mat, mat_norm, 0.0, 1.0, cv::NORM_MINMAX);
      cv::imshow(win_name, mat_norm);
    }
    else
    {
      cv::imshow(win_name, mat);
    }
  }
}


// -----------------------------------------------------------------------------
void Visualizer::showVoronoi(int level,
                             const std::string &win_name,
                             const cv::Mat &skel,
                             const cv::Mat &occupancy,
                             const cv::Point &seed) const
{
  if (visualization_level_ >= level)
  {
    cv::Mat img(skel.size(), CV_8UC3);
    int count = skel.total();
    for (int i = 0; i < count; ++i)
    {
      if (skel.data[i] == 0)
      {
        img.data[i * 3 + 0] = 255;
        img.data[i * 3 + 1] = 0;
        img.data[i * 3 + 2] = 0;
      }
      else
      {
        if (occupancy.data[i] == 0)
        {
          img.data[i * 3 + 0] = 0;
          img.data[i * 3 + 1] = 0;
          img.data[i * 3 + 2] = 0;
        }
        else if (occupancy.data[i] == 100)
        {
          img.data[i * 3 + 0] = 100;
          img.data[i * 3 + 1] = 100;
          img.data[i * 3 + 2] = 100;
        }
        else
        {
          img.data[i * 3 + 0] = 255;
          img.data[i * 3 + 1] = 255;
          img.data[i * 3 + 2] = 255;
        }
      }
    }

    // Visualize the seed
    cv::circle(img, seed, 5, cv::Scalar(0, 0, 255), 3);

    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
    cv::imshow(win_name, img);
  }
}


// -----------------------------------------------------------------------------
void Visualizer::showValueMap(int level,
                              const std::string &win_name,
                              const cv::Mat &values,
                              const cv::Mat &occupancy,
                              const std::vector<cv::Point> &seed_places,
                              const int color_map) const
{
  if (visualization_level_ >= level)
  {
    // Get min and max
    double min;
    double max;
    cv::minMaxLoc(values, &min, &max);

    // Convert value to image
    cv::Mat value_img;
    cv::normalize(values, value_img, 0.0, 255.0, cv::NORM_MINMAX);
    value_img.convertTo(value_img, CV_8U);
    cv::applyColorMap(value_img, value_img, color_map);

    // Threshold with occupancy
    int count = occupancy.total();
    for (int i = 0; i < count; ++i)
    {
      // Make obstacles black
      if (occupancy.data[i] == 0)
      {
        value_img.data[i * 3 + 0] = 0;
        value_img.data[i * 3 + 1] = 0;
        value_img.data[i * 3 + 2] = 0;
      }
      // Make unknown area darker
      else if (occupancy.data[i] == 100)
      {
        value_img.data[i * 3 + 0] *= 0.7;
        value_img.data[i * 3 + 1] *= 0.7;
        value_img.data[i * 3 + 2] *= 0.7;
      }
    }

    // Marked seed places
    for (int i = 0; i < seed_places.size(); ++i)
    {
      cv::circle(value_img, seed_places[i], 5, cv::Scalar(150, 150, 150), 2);
    }

    // Add legend
    cv::Mat legend_img(30, value_img.cols, CV_8U);
    for (int i = 0; i < value_img.cols; ++i)
    {
      cv::Mat col =  legend_img.col(i);
      int val =  static_cast<int>(static_cast<float>(i) * 255.0 /
                                  static_cast<float>(value_img.cols));
      col.setTo(val);
    }
    cv::applyColorMap(legend_img, legend_img, color_map);

    // Add text about min and max
    cv::putText(legend_img,
                str(format("%.3f") % min),
                cv::Point(20, 20),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
    cv::putText(legend_img,
                str(format("%.3f") % ((max - min) / 2.0)),
                cv::Point(legend_img.cols / 2 - 20, 20),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(125, 125, 125));
    cv::putText(legend_img,
                str(format("%.3f") % max),
                cv::Point(legend_img.cols - 60, 20),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0));

    // Show image
    legend_img.push_back(value_img);
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
    cv::imshow(win_name, legend_img);
  }
}

}  // namespace infobot_topo_mapping
