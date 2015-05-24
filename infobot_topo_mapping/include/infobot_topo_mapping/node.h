#ifndef _INFOBOT_TOPO_MAPPING__NODE_
#define _INFOBOT_TOPO_MAPPING__NODE_

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <infobot_topo_msgs/ProcessMetricMap.h>
#include <infobot_topo_msgs/ProcessCurrentMetricMap.h>
#include <infobot_topo_msgs/TopologicalMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include "infobot_topo_mapping/topo_extractor.h"
#include "infobot_topo_mapping/visualizer.h"
#include "infobot_topo_mapping/InfoBotTopoMappingConfig.h"

// #include <string>


namespace infobot_topo_mapping
{


/**
 * Class managing the ROS interface.
 */
class Node
{
 public:
  /** Initializes the node. */
  Node(int argc, char **argv);

  /** Spins, is blocking. */
  void spin();

 private:
  // Node handle
  ros::NodeHandle nh_;

  // Publishers & Subscribers
  ros::Publisher topo_map_pub_;
  ros::Subscriber topo_map_sub_;
  ros::Subscriber metric_map_sub_;
  ros::ServiceServer process_metric_map_srv_;
  ros::ServiceServer process_current_metric_map_srv_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<InfoBotTopoMappingConfig> config_server_;

  // Maps
  nav_msgs::OccupancyGrid::ConstPtr metric_map_;
  infobot_topo_msgs::TopologicalMap::Ptr topo_map_;

  // Algorithms
  Visualizer visualizer_;
  TopoExtractor topo_extractor_;

 private:  // Callbacks
  /** Process metric map service callback */
  bool processMetricMapCb(infobot_topo_msgs::ProcessMetricMap::Request &req,    // NOLINT(runtime/references)
                          infobot_topo_msgs::ProcessMetricMap::Response &res);  // NOLINT(runtime/references)

  /** Process current metric map service callback */
  bool processCurrentMetricMapCb(
      infobot_topo_msgs::ProcessCurrentMetricMap::Request &req,    // NOLINT(runtime/references)
      infobot_topo_msgs::ProcessCurrentMetricMap::Response &res);  // NOLINT(runtime/references)

  /** New topo map callback */
  void topoMapCb(const infobot_topo_msgs::TopologicalMap::ConstPtr &msg);

  /** New metric map callback */
  void metricMapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  /** Dynamic reconfigure callback */
  void reconfigureCallback(const InfoBotTopoMappingConfig &new_config,
                           uint32_t level);


 private:
  // /** Publish result. */
  // void publishObjectDetection(const Detector::Detection &detection,
  //                             const std_msgs::Header &header);
};

}  // namespace infobot_topo_mapping

#endif
