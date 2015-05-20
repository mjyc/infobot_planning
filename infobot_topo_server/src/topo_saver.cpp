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



// /**
//  * @brief Map generation node.
//  */
// class MapGenerator
// {


//     void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
//     {
//       std::string mapdatafile = mapname_ + ".pgm";
//       ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
//       FILE* out = fopen(mapdatafile.c_str(), "w");
//       if (!out)
//       {
//         ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
//         return;
//       }

//       fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
//               map->info.resolution, map->info.width, map->info.height);
//       for(unsigned int y = 0; y < map->info.height; y++) {
//         for(unsigned int x = 0; x < map->info.width; x++) {
//           unsigned int i = x + (map->info.height - y - 1) * map->info.width;
//           if (map->data[i] == 0) { //occ [0,0.1)
//             fputc(254, out);
//           } else if (map->data[i] == +100) { //occ (0.65,1]
//             fputc(000, out);
//           } else { //occ [0.1,0.65]
//             fputc(205, out);
//           }
//         }
//       }

//       fclose(out);


//       std::string mapmetadatafile = mapname_ + ".yaml";
//       ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
//       FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


//       /*
// resolution: 0.100000
// origin: [0.000000, 0.000000, 0.000000]
// #
// negate: 0
// occupied_thresh: 0.65
// free_thresh: 0.196

//        */

//       geometry_msgs::Quaternion orientation = map->info.origin.orientation;
//       tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
//       double yaw, pitch, roll;
//       mat.getEulerYPR(yaw, pitch, roll);

//       fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
//               mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

//       fclose(yaml);

//       ROS_INFO("Done\n");
//       saved_map_ = true;
//     }

//     std::string mapname_;
//     ros::Subscriber map_sub_;
//     bool saved_map_;

// };
