#include "infobot_topo_mapping/node.h"

#include <ros/callback_queue.h>
#include <opencv2/highgui/highgui.hpp>

#include "infobot_topo_mapping/topo_exception.h"

using infobot_topo_msgs::ProcessMetricMap;
using infobot_topo_msgs::ProcessCurrentMetricMap;
using infobot_topo_msgs::TopologicalMap;
using nav_msgs::OccupancyGrid;


namespace infobot_topo_mapping
{

// -----------------------------------------------------------------------------
Node::Node(int argc, char **argv)
    : nh_("~"),
      topo_map_(boost::make_shared<TopologicalMap>()),
      topo_extractor_(topo_map_, visualizer_)
{
  // Advertise and subscribe
  topo_map_pub_ = nh_.advertise<TopologicalMap>("topo_map_out", 1, true);
  topo_map_sub_ = nh_.subscribe("topo_map_in", 1, &Node::topoMapCb, this);
  metric_map_sub_ = nh_.subscribe("metric_map", 1, &Node::metricMapCb, this);
  process_metric_map_srv_ = nh_.advertiseService("process_metric_map",
                                                 &Node::processMetricMapCb, this);
  process_current_metric_map_srv_ = nh_.advertiseService("process_current_metric_map",
                                                         &Node::processCurrentMetricMapCb, this);

  // Comments
  ROS_INFO("Subscribed to topological map topic '%s'.", topo_map_sub_.getTopic().c_str());
  ROS_INFO("Subscribed to metric map topic '%s'.", metric_map_sub_.getTopic().c_str());
  ROS_INFO("Publishing to topological map topic '%s'.", topo_map_pub_.getTopic().c_str());

  // Dynamic reconfigure
  dynamic_reconfigure::Server<InfoBotTopoMappingConfig>::CallbackType config_cb =
      boost::bind(&Node::reconfigureCallback, this, _1, _2);
  config_server_.setCallback(config_cb);
}


// -----------------------------------------------------------------------------
void Node::reconfigureCallback(const InfoBotTopoMappingConfig &new_config,
                               const uint32_t level)
{
  if (level&1)
  {
    ROS_INFO("Setting parameters:");
    ROS_INFO("-> visualization_level: %d", new_config.visualization_level);
    visualizer_.setParams(new_config.visualization_level);
  }

  if (level&2)
  {
    ROS_INFO("Setting algorithm parameters:");
    ROS_INFO("-> min_obstacle_dist: %f", new_config.min_obstacle_dist);
    ROS_INFO("-> optimal_view_dist: %f", new_config.optimal_view_dist);
    ROS_INFO("-> min_voronoi_dist: %f", new_config.min_voronoi_dist);
    ROS_INFO("-> obstacle_cost_scaling: %f", new_config.obstacle_cost_scaling);
    ROS_INFO("-> view_cost_scaling: %f", new_config.view_cost_scaling);
    ROS_INFO("-> center_cost_scaling: %f", new_config.center_cost_scaling);
    ROS_INFO("-> obstacle_cost_weight: %f", new_config.obstacle_cost_weight);
    ROS_INFO("-> view_cost_weight: %f", new_config.view_cost_weight);
    ROS_INFO("-> center_cost_weight: %f", new_config.center_cost_weight);
    topo_extractor_.setCostExtractorParams(new_config.min_obstacle_dist,
                                           new_config.optimal_view_dist,
                                           new_config.min_voronoi_dist,
                                           new_config.obstacle_cost_scaling,
                                           new_config.view_cost_scaling,
                                           new_config.center_cost_scaling,
                                           new_config.obstacle_cost_weight,
                                           new_config.view_cost_weight,
                                           new_config.center_cost_weight);
  }

  if (level&3)
  {
    ROS_INFO("-> gibbs_max_iter: %d", new_config.gibbs_max_iter);
    ROS_INFO("-> gibbs_burnin: %d", new_config.gibbs_burnin);
    topo_extractor_.setPlaceSamplerParams(new_config.gibbs_max_iter,
                                          new_config.gibbs_burnin);
  }
}


// -----------------------------------------------------------------------------
bool Node::processMetricMapCb(ProcessMetricMap::Request &req,
                              ProcessMetricMap::Response &res)
{
  ROS_INFO("Processing given metric map.");
  try
  {
    if (req.map_relations.size() > 0)
      topo_extractor_.processMetricMap(req.metric_map, req.map_relations);
    else
      topo_extractor_.processMetricMap(req.metric_map, req.seed_view);
    ROS_INFO("Finished metric map processing!");
    return true;
  }
  catch(const TopoException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}


// -----------------------------------------------------------------------------
bool Node::processCurrentMetricMapCb(ProcessCurrentMetricMap::Request &req,
                                     ProcessCurrentMetricMap::Response &res)
{
  if (!metric_map_)
  {
    ROS_ERROR("Cannot process the current metric map. No map received yet!");
    return false;
  }

  ROS_INFO("Processing current metric map on topic %s.", metric_map_sub_.getTopic().c_str());
  try
  {
    if (req.map_relations.size() > 0)
      topo_extractor_.processMetricMap(*metric_map_, req.map_relations);
    else
      topo_extractor_.processMetricMap(*metric_map_, req.seed_view);
    ROS_INFO("Finished metric map processing!");

    topo_map_pub_.publish(topo_extractor_.topoMap());
    ROS_INFO("Topological map published!");
    return true;
  }
  catch(const TopoException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}


// -----------------------------------------------------------------
void Node::topoMapCb(const TopologicalMap::ConstPtr &msg)
{
  ROS_INFO("New topological map on the topic.");
  if ((!topo_map_) || (topo_map_->places.empty())
      || (msg->header.seq > topo_map_->header.seq))
  {
    ROS_INFO("Accepting the map as new one.");
    // Make a deep copy of the map to modify locally
    topo_map_ = boost::make_shared<TopologicalMap>(*msg);
    topo_extractor_.setTopoMap(topo_map_);
  }
}


// -----------------------------------------------------------------------------
void Node::metricMapCb(const OccupancyGrid::ConstPtr &msg)
{
  ROS_INFO("Received metric map.");
  metric_map_ = msg;
}


// // -----------------------------------------------------------------
// void Node::publishObjectDetection(const Detector::Detection &detection,
//                                   const std_msgs::Header &header)
// {
//   ROS_INFO("Detection result: %s:%.3f (Votes: [pos=%d, scale=%d, angle=%d])", _objectName.c_str(),
//            detection.confidence, detection.vPos, detection.vScale, detection.vAngle);

//   infobot_categorical_msgs::ObjectDetection3D objDetectionMsg;
//   objDetectionMsg.header = header;
//   objDetectionMsg.is_valid = true;
//   objDetectionMsg.object_name = _objectName;
//   objDetectionMsg.meta_data = "";
//   objDetectionMsg.confidence = detection.confidence;
//   objDetectionMsg.center.x = detection.cx;
//   objDetectionMsg.center.y = detection.cy;
//   objDetectionMsg.center.z = 0.0;
//   objDetectionMsg.size.x = detection.sx;
//   objDetectionMsg.size.y = detection.sy;
//   objDetectionMsg.size.z = 0.0;
//   tf::Quaternion q(0.0, 0.0, detection.angle);
//   objDetectionMsg.orientation.x = q.x();
//   objDetectionMsg.orientation.y = q.y();
//   objDetectionMsg.orientation.z = q.z();
//   objDetectionMsg.orientation.w = q.w();
//   objDetectionMsg.input_topic = _inputScansSub.getTopic();
//   objDetectionMsg.output_topic = _outputScansPub.getTopic();
//   objDetectionMsg.mesh_file = _meshFile;

//   _detectionsPub.publish(objDetectionMsg);
// }


// -----------------------------------------------------------------------------
void Node::spin()
{
  // We do our own spin to also handle visualization events
  while (visualizer_.processEvents())
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }
}

}  // namespace infobot_topo_mapping


// -----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "infobot_topo_mapping");

  infobot_topo_mapping::Node n(argc, argv);
  n.spin();
  return 0;
}
