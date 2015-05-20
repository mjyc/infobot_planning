#ifndef _INFOBOT_TOPO_MAPPING__THINNING_
#define _INFOBOT_TOPO_MAPPING__THINNING_

// OpenCV implementation of the Zhang-Suen thinning algorithm.
// The algorithm itself has been taken from:
// https://github.com/bsdnoobz/zhang-suen-thinning


namespace infobot_topo_mapping
{

  /**
   * Function for thinning the given binary image.
   *
   * Parameters:
   * src  The source image, binary with range = [0,255]
   * dst  The destination image
   */
  void thinning(const cv::Mat& src, cv::Mat& dst);  // NOLINT(runtime/references)

}  // namespace infobot_topo_mapping

#endif
