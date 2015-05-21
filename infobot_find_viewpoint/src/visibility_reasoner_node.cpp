#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <infobot_find_viewpoint_msgs/GetProbabilityOctomapSurface.h>
#include <infobot_find_viewpoint_msgs/GetProbabilityOctomapHeight.h>
#include <infobot_find_viewpoint_msgs/ComputeVisibilityValue.h>
#include <infobot_find_viewpoint_msgs/ComputeVisibilityValues.h>
#include <infobot_topo_msgs/TopologicalMap.h>
#include <infobot_topo_msgs/GetTopologicalMap.h>

#include "./visibility_reasoner_utils.h"


class VisibilityReasonerNode
{
public:
  // Camera parameters are from: http://openkinect.org/wiki/Imaging_Information
  static const double DEFAULT_HORIZONTAL_ANGLE_OF_VIEW;  // in radians
  static const double DEFAULT_VERTICAL_ANGLE_OF_VIEW;  // in radians
  static const double DEFAULT_DEPTH_MAX;  // in m
  static const int DEFAULT_FOV_GRID_RESOLUTION;
  static const double DEFAULT_HEIGHT_MEAN;
  static const double DEFAULT_HEIGHT_VAR;
  static const double DEFAULT_PROSILICA_HEIGHT;

public:
  VisibilityReasonerNode(int argc, char **argv);

  void spin();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle privateNh_;

  ros::ServiceServer computeVisValueSrv_;
  ros::ServiceServer computeVisValuesSrv_;

  ros::Publisher cameraFOVMarkersPub_;
  ros::Publisher visibleOctomapPub_;
  ros::Publisher visValsMarkersPub_;

  double horizontal_angle_of_view_;
  double vertical_angle_of_view_;
  double depth_max_;
  int fov_grid_res_;
  double height_mean_;
  double height_var_;
  double prosilica_height_;

private:
  bool computeVisValueSrvCb(infobot_find_viewpoint_msgs::ComputeVisibilityValue::Request  &req,
                            infobot_find_viewpoint_msgs::ComputeVisibilityValue::Response &res);
  bool computeVisValuesSrvCb(infobot_find_viewpoint_msgs::ComputeVisibilityValues::Request  &req,
                            infobot_find_viewpoint_msgs::ComputeVisibilityValues::Response &res);
};

const double VisibilityReasonerNode::DEFAULT_HORIZONTAL_ANGLE_OF_VIEW = 58.00 * M_PI / 180;  // in radians
const double VisibilityReasonerNode::DEFAULT_VERTICAL_ANGLE_OF_VIEW = 45.00 * M_PI / 180;  // in radians
const double VisibilityReasonerNode::DEFAULT_DEPTH_MAX = 10;  // in m
const int VisibilityReasonerNode::DEFAULT_FOV_GRID_RESOLUTION = 50;
const double VisibilityReasonerNode::DEFAULT_HEIGHT_MEAN = 1.65;
const double VisibilityReasonerNode::DEFAULT_HEIGHT_VAR = 0.1;
const double VisibilityReasonerNode::DEFAULT_PROSILICA_HEIGHT = 1.35;

//----------------------------------------------------------------------
VisibilityReasonerNode::VisibilityReasonerNode(int argc, char **argv):
  nh_(),
  privateNh_("~")
{
  computeVisValueSrv_ = nh_.advertiseService("compute_vis_value",
    &VisibilityReasonerNode::computeVisValueSrvCb, this);
  computeVisValuesSrv_ = nh_.advertiseService("compute_vis_values",
    &VisibilityReasonerNode::computeVisValuesSrvCb, this);

  visibleOctomapPub_ = nh_.advertise<octomap_msgs::Octomap>("visible_octomap", 1, true);
  cameraFOVMarkersPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/vis_reasoner_fov_markers", 10, true);
  visValsMarkersPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/vis_values_markers", 10, true);

  privateNh_.param("horizontal_angle_of_view", horizontal_angle_of_view_, DEFAULT_HORIZONTAL_ANGLE_OF_VIEW);
  privateNh_.param("vertical_angle_of_view", vertical_angle_of_view_, DEFAULT_VERTICAL_ANGLE_OF_VIEW);
  privateNh_.param("depth_max", depth_max_, DEFAULT_DEPTH_MAX);
  privateNh_.param("fov_grid_res", fov_grid_res_, DEFAULT_FOV_GRID_RESOLUTION);
  privateNh_.param("height_mean", height_mean_, DEFAULT_HEIGHT_MEAN);
  privateNh_.param("height_var", height_var_, DEFAULT_HEIGHT_VAR);
  privateNh_.param("prosilica_height", prosilica_height_, DEFAULT_PROSILICA_HEIGHT);
}

//----------------------------------------------------------------------
void VisibilityReasonerNode::spin()
{
  ros::spin();
}

bool VisibilityReasonerNode::computeVisValueSrvCb(
  infobot_find_viewpoint_msgs::ComputeVisibilityValue::Request  &req,
  infobot_find_viewpoint_msgs::ComputeVisibilityValue::Response &res)
{
  // Get ProbabilityOctomap
  ros::ServiceClient getProbOctomapClient;
  octomap_msgs::Octomap poctomap;
  if (req.pmap_to_octomap_mode == req.SURFACE)
  {
    ROS_INFO("Waiting for /get_poctomap_surface service");
    ros::service::waitForService("/get_poctomap_surface");
    ROS_INFO("Waiting for /get_poctomap_surface service done!");
    getProbOctomapClient =
      nh_.serviceClient<infobot_find_viewpoint_msgs::GetProbabilityOctomapSurface>("/get_poctomap_surface");
    infobot_find_viewpoint_msgs::GetProbabilityOctomapSurface getProbOctomapSrv;
    getProbOctomapSrv.request.pmap_filename = req.pmap_filename;
    getProbOctomapSrv.request.pmap_frame_id = req.pmap_frame_id;
    getProbOctomapSrv.request.octomap_filename = req.octomap_filename;
    getProbOctomapSrv.request.octomap_frame_id = req.octomap_frame_id;
    if (getProbOctomapClient.call(getProbOctomapSrv))
      poctomap = getProbOctomapSrv.response.octomap;
    else
      return false;
  }
  else if (req.pmap_to_octomap_mode == req.HEIGHT)
  {
    ROS_INFO("Waiting for /get_poctomap_height service");
    ros::service::waitForService("/get_poctomap_height");
    ROS_INFO("Waiting for /get_poctomap_height service done!");
    getProbOctomapClient =
      nh_.serviceClient<infobot_find_viewpoint_msgs::GetProbabilityOctomapHeight>("/get_poctomap_height");
    infobot_find_viewpoint_msgs::GetProbabilityOctomapHeight getProbOctomapSrv;
    getProbOctomapSrv.request.pmap_filename = req.pmap_filename;
    getProbOctomapSrv.request.pmap_frame_id = req.pmap_frame_id;
    getProbOctomapSrv.request.octomap_filename = req.octomap_filename;
    getProbOctomapSrv.request.octomap_frame_id = req.octomap_frame_id;
    getProbOctomapSrv.request.height_mean = height_mean_;
    getProbOctomapSrv.request.height_var = height_var_;
    if (getProbOctomapClient.call(getProbOctomapSrv))
      poctomap = getProbOctomapSrv.response.octomap;
    else
      return false;
  }
  else
  {
    ROS_ERROR("Unknown mode=%d", req.pmap_to_octomap_mode);
    return false;
  }
  std::string frame_id = poctomap.header.frame_id;

  // Publish FOV (+Grid) Markers
  std::vector<geometry_msgs::Pose> fovEdgePoses;
  computeFOVEgdePoses(req.camera_pose, horizontal_angle_of_view_, vertical_angle_of_view_, fovEdgePoses);
  double lineOfSightLen = (depth_max_ / cos(horizontal_angle_of_view_ / 2.0)) / cos(vertical_angle_of_view_ / 2.0);
  visualization_msgs::MarkerArray marrayFOVEdges = createFOVEdgeMarkers(frame_id, fovEdgePoses, lineOfSightLen);

  std::vector<geometry_msgs::Point> fovGridPoints;
  int gridResolution = 10;
  computeFOVGridPoints(fovEdgePoses, gridResolution, fovGridPoints);
  visualization_msgs::MarkerArray marrayFOVGrid = createFOVGridMarkers(frame_id, fovGridPoints);

  visualization_msgs::MarkerArray marray;
  marray = marrayFOVEdges;
  marray.markers.insert(marray.markers.end(), marrayFOVGrid.markers.begin(), marrayFOVGrid.markers.end());
  cameraFOVMarkersPub_.publish(marray);

  // Compute Visibility Value
  octomap::OcTree* octree = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(poctomap);
  if (tree)
  {
    octree = dynamic_cast<octomap::OcTree*>(tree);
  }
  else
  {
    return false;
  }

  // double minX, minY, minZ, maxX, maxY, maxZ;
  // octree->getMetricMin(minX, minY, minZ);
  // octree->getMetricMax(maxX, maxY, maxZ);
  // std::cout << minX << " " << minY << " " << minZ << " " << maxX << " " << maxY << " " << maxZ << " " << std::endl;

  double visValue = 0.0;
  octomap::KeySet visibleCells;
  if (!computeVisibilityValue(req.camera_pose, horizontal_angle_of_view_, vertical_angle_of_view_, depth_max_, frame_id,
      fov_grid_res_, visibleCells, visValue, octree))
  {
    delete octree;
    return false;
  }

  // Publish visible cells as an octomap msg

  for (octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it)
  {
    if (visibleCells.find(it.getKey()) != visibleCells.end())
    {
      it->setValue(octree->getClampingThresMaxLog());
    }
    else
    {
      it->setValue(octree->getClampingThresMinLog());
    }
  }
  octomap_msgs::Octomap visibleOctomap;
  visibleOctomap.header.frame_id = frame_id;
  visibleOctomap.header.stamp = ros::Time::now();
  if (!octomap_msgs::fullMapToMsg(*octree, visibleOctomap))
    ROS_ERROR("Error serializing OctoMap. Failed to publish visible cells.");
  else
     visibleOctomapPub_.publish(visibleOctomap);

  delete octree;
  res.vis_value = visValue;
  return true;
}

bool VisibilityReasonerNode::computeVisValuesSrvCb(
  infobot_find_viewpoint_msgs::ComputeVisibilityValues::Request  &req,
  infobot_find_viewpoint_msgs::ComputeVisibilityValues::Response &res)
{
  // Get ProbabilityOctomap
  ros::ServiceClient getProbOctomapClient;
  octomap_msgs::Octomap poctomap;
  if (req.pmap_to_octomap_mode == req.SURFACE)
  {
    ROS_INFO("Waiting for /get_poctomap_surface service");
    ros::service::waitForService("/get_poctomap_surface");
    ROS_INFO("Waiting for /get_poctomap_surface service done!");
    getProbOctomapClient =
      nh_.serviceClient<infobot_find_viewpoint_msgs::GetProbabilityOctomapSurface>("/get_poctomap_surface");
    infobot_find_viewpoint_msgs::GetProbabilityOctomapSurface getProbOctomapSrv;
    getProbOctomapSrv.request.pmap_filename = req.pmap_filename;
    getProbOctomapSrv.request.pmap_frame_id = req.pmap_frame_id;
    getProbOctomapSrv.request.octomap_filename = req.octomap_filename;
    getProbOctomapSrv.request.octomap_frame_id = req.octomap_frame_id;
    if (getProbOctomapClient.call(getProbOctomapSrv))
      poctomap = getProbOctomapSrv.response.octomap;
    else
      return false;
  }
  else if (req.pmap_to_octomap_mode == req.HEIGHT)
  {
    ROS_INFO("Waiting for /get_poctomap_height service");
    ros::service::waitForService("/get_poctomap_height");
    ROS_INFO("Waiting for /get_poctomap_height service done!");
    getProbOctomapClient =
      nh_.serviceClient<infobot_find_viewpoint_msgs::GetProbabilityOctomapHeight>("/get_poctomap_height");
    infobot_find_viewpoint_msgs::GetProbabilityOctomapHeight getProbOctomapSrv;
    getProbOctomapSrv.request.pmap_filename = req.pmap_filename;
    getProbOctomapSrv.request.pmap_frame_id = req.pmap_frame_id;
    getProbOctomapSrv.request.octomap_filename = req.octomap_filename;
    getProbOctomapSrv.request.octomap_frame_id = req.octomap_frame_id;
    getProbOctomapSrv.request.height_mean = height_mean_;
    getProbOctomapSrv.request.height_var = height_var_;
    if (getProbOctomapClient.call(getProbOctomapSrv))
      poctomap = getProbOctomapSrv.response.octomap;
    else
      return false;
  }
  else
  {
    ROS_ERROR("Unknown mode=%d", req.pmap_to_octomap_mode);
    return false;
  }
  std::string frame_id = poctomap.header.frame_id;

  // TODO(mjyc): check frame_id of topomap.
  infobot_topo_msgs::TopologicalMap topomap;
  ROS_INFO("Waiting for /get_topomap service");
  ros::service::waitForService("/get_topomap");
  ROS_INFO("Waiting for /get_topomap service done!");
  ros::ServiceClient getTopoMapClient = nh_.serviceClient<infobot_topo_msgs::GetTopologicalMap>("/get_topomap");
  infobot_topo_msgs::GetTopologicalMap getTopoMapSrv;
  getTopoMapSrv.request.filename = req.topomap_filename;
  getTopoMapSrv.request.frame_id = req.topomap_frame_id;
  if (getTopoMapClient.call(getTopoMapSrv))
    topomap = getTopoMapSrv.response.topomap;
  else
    return false;

  // Compute Visibility Value
  octomap::OcTree* octree = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(poctomap);
  if (tree)
  {
    octree = dynamic_cast<octomap::OcTree*>(tree);
  }
  else
  {
    return false;
  }

  // double minX, minY, minZ, maxX, maxY, maxZ;
  // octree->getMetricMin(minX, minY, minZ);
  // octree->getMetricMax(maxX, maxY, maxZ);
  // std::cout << minX << " " << minY << " " << minZ << " " << maxX << " " << maxY << " " << maxZ << " " << std::endl;

  for (int i = 0; i < topomap.places.size(); ++i)
  {
    for (int j = 0; j < topomap.places[i].views.size(); ++j)
    {
      geometry_msgs::Pose pose;
      pose.position.x = topomap.places[i].views[j].pose.x;
      pose.position.y = topomap.places[i].views[j].pose.y;
      pose.position.z = prosilica_height_;
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, topomap.places[i].views[j].pose.theta);

      double visValue = 0.0;
      octomap::KeySet visibleCells;
      if (!computeVisibilityValue(pose, horizontal_angle_of_view_, vertical_angle_of_view_, depth_max_, frame_id,
          fov_grid_res_, visibleCells, visValue, octree))
      {
        ROS_ERROR("Failed to compute a visibility value.");
        delete octree;
        return false;
      }

      ROS_INFO("topomap_id=%s, place_id=%d, view_id=%d, vis_value=%f",
               topomap.topo_map_id.c_str(), topomap.places[i].place_id, topomap.places[i].views[j].view_id,
               visValue);

      res.poses.push_back(pose);
      res.vis_values.push_back(visValue);
    }
  }
  delete octree;
  double maxVisValue = *std::max_element(res.vis_values.begin(), res.vis_values.end());
  int maxVisibilityValueIdx = std::distance(res.vis_values.begin(),
    std::max_element(res.vis_values.begin(), res.vis_values.end()));
  res.max_vis_value_index = maxVisibilityValueIdx;

  // Publish markers
  visualization_msgs::MarkerArray marray = createVisValsMarkers(
    frame_id, res.poses, res.vis_values, res.max_vis_value_index);
  visValsMarkersPub_.publish(marray);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visibility_reasoner");

  VisibilityReasonerNode n(argc, argv);
  n.spin();

  return 0;
}
