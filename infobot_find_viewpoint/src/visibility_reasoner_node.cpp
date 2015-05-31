#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <infobot_find_viewpoint/InfoBotFindViewpointVisConfig.h>
#include <infobot_find_viewpoint_msgs/GetProbabilityOctomapSurface.h>
#include <infobot_find_viewpoint_msgs/GetProbabilityOctomapHeight.h>
#include <infobot_find_viewpoint_msgs/ComputeVisibilityValue.h>
#include <infobot_find_viewpoint_msgs/ComputeVisibilityValues.h>

#include "./visibility_reasoner_utils.h"


class VisibilityReasonerNode
{
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
  ros::Publisher maxVisValMarkersPub_;

  dynamic_reconfigure::Server<infobot_find_viewpoint::InfoBotFindViewpointVisConfig> config_server_;

  double horizontal_angle_of_view_;
  double vertical_angle_of_view_;
  double depth_max_;
  int fov_grid_res_;
  double dist_factor_;
  double height_mean_;
  double height_var_;

private:
  void reconfigureCallback(const infobot_find_viewpoint::InfoBotFindViewpointVisConfig &new_config,
                           uint32_t level);
  bool computeVisValueSrvCb(infobot_find_viewpoint_msgs::ComputeVisibilityValue::Request  &req,
                            infobot_find_viewpoint_msgs::ComputeVisibilityValue::Response &res);
  bool computeVisValuesSrvCb(infobot_find_viewpoint_msgs::ComputeVisibilityValues::Request  &req,
                             infobot_find_viewpoint_msgs::ComputeVisibilityValues::Response &res);
};

// const double VisibilityReasonerNode::DEFAULT_HORIZONTAL_ANGLE_OF_VIEW = 96.80 * M_PI / 180;  // in radians
// const double VisibilityReasonerNode::DEFAULT_VERTICAL_ANGLE_OF_VIEW = 79.40 * M_PI / 180;  // in radians
// const double VisibilityReasonerNode::DEFAULT_DEPTH_MAX = 10;  // in m
// const int VisibilityReasonerNode::DEFAULT_FOV_GRID_RESOLUTION = 50;
// const double VisibilityReasonerNode::DEFAULT_HEIGHT_MEAN = 1.65;
// const double VisibilityReasonerNode::DEFAULT_HEIGHT_VAR = 0.05;


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
  maxVisValMarkersPub_ = nh_.advertise<visualization_msgs::Marker>("/max_vis_values_marker", 10, true);

  dynamic_reconfigure::Server<infobot_find_viewpoint::InfoBotFindViewpointVisConfig>::CallbackType config_cb =
      boost::bind(&VisibilityReasonerNode::reconfigureCallback, this, _1, _2);
  config_server_.setCallback(config_cb);
}

//----------------------------------------------------------------------
void VisibilityReasonerNode::spin()
{
  ros::spin();
}

void VisibilityReasonerNode::reconfigureCallback(
  const infobot_find_viewpoint::InfoBotFindViewpointVisConfig &new_config,
  const uint32_t level)
{
  ROS_INFO("Setting parameters:");
  ROS_INFO("-> horizontal_angle_of_view: %f", new_config.horizontal_angle_of_view);
  ROS_INFO("-> vertical_angle_of_view: %f", new_config.vertical_angle_of_view);
  ROS_INFO("-> depth_max: %f", new_config.depth_max);
  ROS_INFO("-> fov_grid_res: %d", new_config.fov_grid_res);
  ROS_INFO("-> dist_factor: %f", new_config.dist_factor);
  ROS_INFO("-> height_mean: %f", new_config.height_mean);
  ROS_INFO("-> height_var: %f", new_config.height_var);

  horizontal_angle_of_view_ = new_config.horizontal_angle_of_view;
  vertical_angle_of_view_ = new_config.vertical_angle_of_view;
  depth_max_ = new_config.depth_max;
  fov_grid_res_ = new_config.fov_grid_res;
  dist_factor_ = new_config.dist_factor;
  height_mean_ = new_config.height_mean;
  height_var_ = new_config.height_var;
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
  if (!computeVisibilityValue(req.camera_pose, horizontal_angle_of_view_, vertical_angle_of_view_, depth_max_,
                              fov_grid_res_, dist_factor_, visibleCells, visValue, octree))
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
  // Get ProbabilityOctomap.
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
  // set frame_id
  std::string frame_id = poctomap.header.frame_id;

  // Compute Visibility Value.
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
  // for debugging
  // double minX, minY, minZ, maxX, maxY, maxZ;
  // octree->getMetricMin(minX, minY, minZ);
  // octree->getMetricMax(maxX, maxY, maxZ);
  // std::cout << minX << " " << minY << " " << minZ << " " << maxX << " " << maxY << " " << maxZ << " " << std::endl;

  for (int i = 0; i < req.camera_poses.size(); ++i)
  {
    geometry_msgs::Pose pose = req.camera_poses[i];
    pose.position.x = pose.position.x;
    pose.position.y = pose.position.y;
    pose.position.z = pose.position.z;
    pose.orientation = pose.orientation;

    double visValue = 0.0;
    octomap::KeySet visibleCells;
    if (!computeVisibilityValue(pose, horizontal_angle_of_view_, vertical_angle_of_view_, depth_max_,
                                fov_grid_res_, dist_factor_, visibleCells, visValue, octree))
    {
      ROS_ERROR("Failed to compute a visibility value.");
      delete octree;
      return false;
    }

    ROS_DEBUG("vis_value=%f", visValue);

    res.vis_values.push_back(visValue);
  }
  delete octree;

  // Publish markers
  visualization_msgs::MarkerArray marray = createVisValsMarkers(frame_id, req.camera_poses, res.vis_values);
  visValsMarkersPub_.publish(marray);
  maxVisValMarkersPub_.publish(createMaxVisValMarker(frame_id, req.camera_poses, res.vis_values));

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visibility_reasoner");

  VisibilityReasonerNode n(argc, argv);
  n.spin();

  return 0;
}
