#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <infobot_topo_msgs/TopologicalMap.h>
#include <infobot_topo_msgs/TopologicalPlace.h>
#include <infobot_topo_msgs/TopologicalView.h>

#include <string>

namespace infobot_topo_mapping
{

using infobot_topo_msgs::TopologicalMap;
using infobot_topo_msgs::TopologicalPlace;
using infobot_topo_msgs::TopologicalView;
class TopoServer
{
 public:
  // -----------------------------------------------------------------------------
  explicit TopoServer(const std::string &map_name)
      : nh_("~"),
        map_name_(map_name)
  {
    // Advertise latched publisher
    topo_map_pub_ = nh_.advertise<TopologicalMap>("topo_map", 1,  true);
    ROS_INFO("Publishing a topological map to topic '%s'", topo_map_pub_.getTopic().c_str());

    publishTopoMap();
  }

  // -----------------------------------------------------------------------------
  void publishTopoMap()
  {
    ROS_INFO("Loading topological map from file '%s'",  map_name_.c_str());
    TopologicalMap topo_map;

    // Load from file
    YAML::Node main_node = YAML::LoadFile(map_name_);
    topo_map.topo_map_id = main_node["id"].as<std::string>();
    topo_map.header.seq = 1;
    topo_map.header.stamp = ros::Time::now();
    topo_map.header.frame_id = "map";  // This just helps when displaying things in RViz

    // Places
    YAML::Node places_node =  main_node["places"];
    if (places_node.size() == 0)
    {
      ROS_ERROR("No places found in the topo map!");
      return;
    }
    for (int i = 0; i < places_node.size(); i++)
    {
      YAML::Node place_node = places_node[i];
      TopologicalPlace place;
      place.topo_map_id = topo_map.topo_map_id;
      place.metric_map_id = place_node["metric_map_id"].as<std::string>();
      place.place_id = place_node["id"].as<int>();
      YAML::Node views_node =  place_node["views"];
      if (views_node.size() == 0)
      {
        ROS_ERROR("Place %d does not have any views!", place.place_id);
        return;
      }
      for (int j = 0; j < views_node.size(); j++)
      {
        YAML::Node view_node = views_node[j];
        TopologicalView view;
        view.topo_map_id = topo_map.topo_map_id;
        view.metric_map_id = place.metric_map_id;
        view.view_id = view_node["id"].as<int>();
        view.pose.x = view_node["x"].as<double>();
        view.pose.y = view_node["y"].as<double>();
        view.pose.theta = view_node["theta"].as<double>();
        place.views.push_back(view);
      }
      topo_map.places.push_back(place);
    }

    // Publish
    topo_map_pub_.publish(topo_map);
    ROS_INFO("Topological map with ID: %s and %d places published!",
             topo_map.topo_map_id.c_str(),
             topo_map.places.size());
  }

  // -----------------------------------------------------------------------------
  void spin()
  {
    ros::spin();
  }

 private:
  // Node handle
  ros::NodeHandle nh_;

  // Publishers & Subscribers
  ros::Publisher topo_map_pub_;

  // Map name
  std::string map_name_;
};

}  // namespace infobot_topo_mapping


// -----------------------------------------------------------------------------
void printUsage()
{
  std::cout << "Usage:" << std::endl;
  std::cout <<  "  topo_server <topo_map_name> [ROS remapping args]" << std::endl;
}


// -----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "infobot_topo_saver");

  // Parse args
  if (argc < 2)
  {
    printUsage();
    return 1;
  }

  std::string map_name(argv[1]);

  infobot_topo_mapping::TopoServer n(map_name);
  n.spin();

  return 0;
}
