#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


//----------------------------------------------------------------------
// Color Utils
//----------------------------------------------------------------------

// MATLAB Jet Color Palette
// From http://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
// r, g, b are in [0.0,1.0]
inline void colorMATLABJetPalette(double v, double vmin, double vmax, double& r, double& g, double& b)
{
  r = 1.0;
  g = 1.0;
  b = 1.0;

  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv))
  {
    r = 0;
    g = 4 * (v - vmin) / dv;
  }
  else if (v < (vmin + 0.5 * dv))
  {
    r = 0;
    b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  }
  else if (v < (vmin + 0.75 * dv))
  {
    r = 4 * (v - vmin - 0.5 * dv) / dv;
    b = 0;
  }
  else
  {
    g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    b = 0;
  }
}


//----------------------------------------------------------------------
// Marker Utils
//----------------------------------------------------------------------

inline visualization_msgs::Marker createMarker(const std::string &frameId,
    const std::string &ns,
    const int id,
    const int type,
    const geometry_msgs::Pose &pose,
    const geometry_msgs::Vector3 &scale,
    const std_msgs::ColorRGBA &color)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;

  return marker;
}

inline visualization_msgs::MarkerArray createFOVEdgeMarkers(
  const std::string &frameId,
  const std::vector<geometry_msgs::Pose> &fovEdgePoses,
  double lineOfSightLen)
{
  visualization_msgs::MarkerArray marray;

  if (fovEdgePoses.size() != 4)
  {
    ROS_ERROR_STREAM("Input fovEdgePoses size is " << fovEdgePoses.size() << ". It must be 4.");
    return marray;
  }

  int type = visualization_msgs::Marker::ARROW;

  geometry_msgs::Vector3 scale;
  scale.x = lineOfSightLen;
  scale.y = 0.05;
  scale.z = 0.05;

  // DEBUG MODE
  // std_msgs::ColorRGBA color;
  // color.r = 1.0;
  // color.g = 0.0;
  // color.b = 0.0;
  // color.a = 1.0;
  // std_msgs::ColorRGBA color1;
  // color1.r = 0.0;
  // color1.g = 1.0;
  // color1.b = 0.0;
  // color1.a = 1.0;
  // std_msgs::ColorRGBA color2;
  // color2.r = 0.0;
  // color2.g = 0.0;
  // color2.b = 1.0;
  // color2.a = 1.0;
  // std_msgs::ColorRGBA color3;
  // color3.r = 1.0;
  // color3.g = 1.0;
  // color3.b = 1.0;
  // color3.a = 1.0;

  // std::string ns = "fov_edge";
  // marray.markers.push_back(createMarker(frameId, ns, 0, type, fovEdgePoses[0], scale, color));
  // marray.markers.push_back(createMarker(frameId, ns, 1, type, fovEdgePoses[1], scale, color1));
  // marray.markers.push_back(createMarker(frameId, ns, 2, type, fovEdgePoses[2], scale, color2));
  // marray.markers.push_back(createMarker(frameId, ns, 3, type, fovEdgePoses[3], scale, color3));

  // Visualization
  std_msgs::ColorRGBA color;
  color.r = 0.247;
  color.g = 0.0;
  color.b = 0.490;
  color.a = 1.0;

  std::string ns = "fov_edge";
  marray.markers.push_back(createMarker(frameId, ns, 0, type, fovEdgePoses[0], scale, color));
  marray.markers.push_back(createMarker(frameId, ns, 1, type, fovEdgePoses[1], scale, color));
  marray.markers.push_back(createMarker(frameId, ns, 2, type, fovEdgePoses[2], scale, color));
  marray.markers.push_back(createMarker(frameId, ns, 3, type, fovEdgePoses[3], scale, color));

  return marray;
}

inline visualization_msgs::MarkerArray createFOVGridMarkers(
  const std::string &frameId,
  const std::vector<geometry_msgs::Point> &fovEdgePoints)
{
  visualization_msgs::MarkerArray marray;

  int type = visualization_msgs::Marker::SPHERE;

  geometry_msgs::Vector3 scale;
  scale.x = 0.05;
  scale.y = 0.05;
  scale.z = 0.05;

  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;

  std::string ns = "fov_grid";
  for (int i = 0; i < fovEdgePoints.size(); ++i)
  {
    geometry_msgs::Pose pose;
    pose.position = fovEdgePoints[i];
    marray.markers.push_back(createMarker(frameId, ns, i, type, pose, scale, color));
  }

  return marray;
}

inline visualization_msgs::MarkerArray createVisValsMarkers(
  const std::string &frameId,
  const std::vector<geometry_msgs::Pose> &poses,
  const std::vector<double> &vis_values)
{
  visualization_msgs::MarkerArray marray;

  // check args
  if (poses.size() != vis_values.size())
  {
    ROS_ERROR_STREAM(
      "Input poses and vis_values have different sizes. poses.size()=" << poses.size() << ", vis_values.size()=" <<
      vis_values.size());
    return marray;
  }
  else if (poses.size() == 0)
  {
    return marray;
  }
  int maxVisValueIndex = std::distance(vis_values.begin(), std::max_element(vis_values.begin(), vis_values.end()));

  int type = visualization_msgs::Marker::ARROW;

  geometry_msgs::Vector3 scale;
  scale.x = 0.5;
  scale.y = 0.05;
  scale.z = 0.05;

  std_msgs::ColorRGBA color;
  color.a = 1.0;

  std::string ns = "vis_values";
  double maxVisibilityValue = vis_values[maxVisValueIndex];
  for (int i = 0; i < vis_values.size(); ++i)
  {
    double v = vis_values[i];
    if (maxVisibilityValue != 0.0)
      v /= maxVisibilityValue;

    double r, g, b;
    colorMATLABJetPalette(v, 0.0, 1.0, r, g, b);
    color.r = r;
    color.g = g;
    color.b = b;

    marray.markers.push_back(createMarker(frameId, ns, i, type, poses[i], scale, color));
  }

  return marray;
}

inline visualization_msgs::Marker createMaxVisValMarker(
  const std::string &frameId,
  const std::vector<geometry_msgs::Pose> &poses,
  const std::vector<double> &vis_values)
{
  // check args
  if (poses.size() != vis_values.size())
  {
    ROS_ERROR_STREAM(
      "Input poses and vis_values have different sizes. poses.size()=" << poses.size() << ", vis_values.size()=" <<
      vis_values.size());
    visualization_msgs::Marker dummy_marker;
    return dummy_marker;
  }
  int maxVisValueIndex = std::distance(vis_values.begin(), std::max_element(vis_values.begin(), vis_values.end()));

  int type = visualization_msgs::Marker::ARROW;

  geometry_msgs::Vector3 scale;
  scale.x = 0.5;
  scale.y = 0.2;
  scale.z = 0.2;

  std_msgs::ColorRGBA color;
  color.a = 1.0;

  std::string ns = "max_vis_value";

  double r, g, b;
  colorMATLABJetPalette(1.0, 0.0, 1.0, r, g, b);
  color.r = r;
  color.g = g;
  color.b = b;

  return createMarker(frameId, ns, 0, type, poses[maxVisValueIndex], scale, color);
}


//----------------------------------------------------------------------
// 3D Utils
//----------------------------------------------------------------------

inline void computeFOVEgdePoses(const geometry_msgs::Pose &cameraPose,
    const double horizontalAngleOfView,
    const double verticalAngleOfView,
    std::vector<geometry_msgs::Pose> &fovEdgePoses)
{
  double dRoll = 0.0;
  double dYaw = horizontalAngleOfView / 2.0;
  double dPitch = verticalAngleOfView / 2.0;

  tf::Quaternion cameraQuat;
  tf::quaternionMsgToTF(cameraPose.orientation, cameraQuat);

  geometry_msgs::Pose p1;
  p1.position = cameraPose.position;
  tf::quaternionTFToMsg(cameraQuat * tf::createQuaternionFromRPY(dRoll, dPitch, dYaw), p1.orientation);
  geometry_msgs::Pose p2;
  p2.position = cameraPose.position;
  tf::quaternionTFToMsg(cameraQuat * tf::createQuaternionFromRPY(dRoll, dPitch, -1.0 * dYaw), p2.orientation);
  geometry_msgs::Pose p3;
  p3.position = cameraPose.position;
  tf::quaternionTFToMsg(cameraQuat * tf::createQuaternionFromRPY(dRoll, -1.0 * dPitch, dYaw), p3.orientation);
  geometry_msgs::Pose p4;
  p4.position = cameraPose.position;
  tf::quaternionTFToMsg(cameraQuat * tf::createQuaternionFromRPY(dRoll, -1.0 * dPitch, -1.0 * dYaw), p4.orientation);

  fovEdgePoses.resize(4);
  fovEdgePoses[0] = p1;
  fovEdgePoses[1] = p2;
  fovEdgePoses[2] = p3;
  fovEdgePoses[3] = p4;
}

inline void computeFOVGridPoints(const std::vector<geometry_msgs::Pose> &fovEdgePoses,
    const int fovGridResolution,
    std::vector<geometry_msgs::Point> &fovEdgePoints)
{
  if (fovEdgePoses.size() != 4)
  {
    ROS_ERROR_STREAM("Input fovEdgePoses size is " << fovEdgePoses.size() << ". It must be 4.");
    return;
  }

  std::vector<tf::Vector3> fovEdgeVectors(4);
  for (int i = 0; i < fovEdgePoses.size(); ++i)
  {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(fovEdgePoses[i].orientation, quat);
    double roll, pitch, yaw, x, y, z;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    x = cos(pitch) * cos(yaw);
    y = cos(-1.0 * pitch) * sin(yaw);
    z = sin(-1.0 * pitch);
    fovEdgeVectors[i] = tf::Vector3(x, y, z);
  }

  tf::Vector3 dX = (fovEdgeVectors[1] - fovEdgeVectors[0]) / (1.0 * fovGridResolution);
  tf::Vector3 dY = (fovEdgeVectors[2] - fovEdgeVectors[0]) / (1.0 * fovGridResolution);
  // std::cout << dX.x() << " " << dX.y() << " " << dX.z() << " " << std::endl;
  // std::cout << dY.x() << " " << dY.y() << " " << dY.z() << " " << std::endl;

  fovEdgePoints.clear();
  for (int i = 0; i < fovGridResolution + 1; ++i)
  {
    for (int j = 0; j < fovGridResolution + 1; ++j)
    {
      tf::Vector3 v = fovEdgeVectors[0] + dX * (1.0 * j) + dY * (1.0 * i);
      geometry_msgs::Point p;
      p.x = v.x() + fovEdgePoses[0].position.x;
      p.y = v.y() + fovEdgePoses[0].position.y;
      p.z = v.z() + fovEdgePoses[0].position.z;
      // std::cout << p.x << " " << p.y << " " << p.z << " " << std::endl;
      fovEdgePoints.push_back(p);
    }
  }
}

inline bool computeVisibilityValue(const geometry_msgs::Pose &cameraPose,
    const double horizontalAngleOfView,
    const double verticalAngleOfView,
    const double depthMax,
    const int fovGridResolution,
    const double distanceFactorA,
    const double distanceFactorB,
    const double angleFactor,
    octomap::KeySet &visibleCells,
    double &visibilityValue,
    octomap::OcTree* octree)
{
  std::vector<geometry_msgs::Pose> fovEdgePoses;
  computeFOVEgdePoses(cameraPose, horizontalAngleOfView, verticalAngleOfView, fovEdgePoses);

  std::vector<geometry_msgs::Point> fovGridPointsMsg;
  computeFOVGridPoints(fovEdgePoses, fovGridResolution, fovGridPointsMsg);

  octomap::point3d cameraOrigin(cameraPose.position.x, cameraPose.position.y, cameraPose.position.z);

  std::vector<octomap::point3d> fovGridPoints(fovGridPointsMsg.size());
  for (int i = 0; i < fovGridPointsMsg.size(); ++i)
  {
    fovGridPoints[i] = octomap::point3d(fovGridPointsMsg[i].x, fovGridPointsMsg[i].y, fovGridPointsMsg[i].z) -
      cameraOrigin;
  }

  visibilityValue = 0.0;
  int probValOffset = static_cast<int>(octree->getClampingThresMaxLog() + 0.5);
  double cameraRotYaw = tf::getYaw(cameraPose.orientation);
  for (int i = 0; i < fovGridPoints.size(); i++)
  {
    octomap::point3d result;
    octomap::OcTreeKey resultKey;
    octree->castRay(cameraOrigin, fovGridPoints[i], result, true, depthMax);
    if (!octree->coordToKeyChecked(result, resultKey))
    {
      ROS_WARN_STREAM("Could not create a OcTreeKey at " << result << ". Continuing...");
      continue;
    }

    octomap::OcTreeNode* node = octree->search(resultKey);
    if (node)
    {
      if (node->getValue() > probValOffset)
      {
        double curVisibilityValue = node->getValue() - probValOffset;
        double dw = -1.0 * distanceFactorA * fabs(cameraOrigin.distance(result) - distanceFactorB);
        double angle = atan2(result.y() - cameraOrigin.y(), result.x() - cameraOrigin.x());
        double angle_dist = fabs(std::min((2 * M_PI) - fabs(angle - cameraRotYaw), fabs(angle - cameraRotYaw)) / M_PI);
        double aw = -1.0 * angleFactor * angle_dist;
        double w = dw + aw + 1.0;
        curVisibilityValue *= w;

        visibilityValue += curVisibilityValue;
        node->setValue(curVisibilityValue + (1.0 * probValOffset));  // for debugging
        visibleCells.insert(resultKey);
      }
    }
  }

  return true;
}
