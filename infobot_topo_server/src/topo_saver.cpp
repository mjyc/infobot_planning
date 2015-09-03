#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <infobot_topo_msgs/TopologicalMap.h>
#include <infobot_topo_msgs/TopologicalPlace.h>
#include <infobot_topo_msgs/TopologicalView.h>

#include <string>
#include <fstream>

namespace infobot_topo_mapping
{

using infobot_topo_msgs::TopologicalMap;
using infobot_topo_msgs::TopologicalPlace;
using infobot_topo_msgs::TopologicalView;

class TopoSaver
{
 public:
  // -----------------------------------------------------------------------------
  explicit TopoSaver(const std::string &map_name)
      : nh_("~"),
        map_name_(map_name),
        map_saved_(false)
  {
    // Subscribe
    topo_map_sub_ = nh_.subscribe("topo_map", 1, &TopoSaver::topoMapCb, this);
    ROS_INFO("Waiting for a topological map on topic '%s'.", topo_map_sub_.getTopic().c_str());
  }

  // -----------------------------------------------------------------
  void topoMapCb(const TopologicalMap::ConstPtr &msg)
  {
    ROS_INFO("Received topological map with ID: %s.",  msg->topo_map_id.c_str());

    // Construct filename
    if (map_name_.empty())
      map_name_ = msg->topo_map_id + ".yaml";

    // Create YAML
    YAML::Node main_node;
    main_node["id"] =  msg->topo_map_id;
    for (int i = 0; i < msg->places.size(); ++i)
    {
      const TopologicalPlace &place = msg->places[i];
      YAML::Node place_node;
      place_node["id"] = place.place_id;
      place_node["metric_map_id"] = place.metric_map_id;
      for (int i = 0; i < place.views.size(); ++i)
      {
        const TopologicalView &view = place.views[i];
        YAML::Node view_node;
        view_node["id"] = view.view_id;
        view_node["x"] = view.pose.x;
        view_node["y"] = view.pose.y;
        view_node["theta"] = view.pose.theta;
        place_node["views"].push_back(view_node);
      }
      main_node["places"].push_back(place_node);
    }

    // Save
    std::ofstream fout(map_name_.c_str());
    fout << main_node;

    ROS_INFO("Topological map saved!");
    map_saved_ = true;
  }

  // -----------------------------------------------------------------------------
  void spin()
  {
    ros::Rate r(10);  // 10 hz
    while (!map_saved_ && ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  }

 private:
  // Node handle
  ros::NodeHandle nh_;

  // Publishers & Subscribers
  ros::Subscriber topo_map_sub_;

  // Map name
  std::string map_name_;

  // True if map saved
  bool map_saved_;
};

}  // namespace infobot_topo_mapping


// -----------------------------------------------------------------------------
void printUsage()
{
  std::cout << "Usage:" << std::endl;
  std::cout <<  "  topo_saver -h" << std::endl;
  std::cout <<  "  topo_saver [-f <topo_map_file_name>] [ROS remapping args]" << std::endl;
}


// -----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "infobot_topo_saver");

  // Parse args
  std::string map_name;

  for (int i = 1; i < argc; i++)
  {
    if (!strcmp(argv[i], "-h"))
    {
      printUsage();
      return 0;
    }
    else if (!strcmp(argv[i], "-f"))
    {
      if (++i < argc)
        map_name = argv[i];
      else
      {
        printUsage();
        return 1;
      }
    }
    else
    {
      printUsage();
      return 1;
    }
  }

  // Start the saver
  infobot_topo_mapping::TopoSaver n(map_name);
  n.spin();

  return 0;
}
