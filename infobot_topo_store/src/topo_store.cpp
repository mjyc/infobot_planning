#include <string>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <infobot_topo_msgs/TopologicalMap.h>
#include <infobot_topo_msgs/TopologicalPlace.h>
#include <infobot_topo_msgs/TopologicalView.h>
#include <infobot_topo_msgs/ApplyRectangleFilter.h>
#include <infobot_topo_msgs/ListTopologicalMapFiles.h>
#include <infobot_topo_msgs/PublishTopologicalMap.h>
#include <infobot_topo_msgs/GetTopologicalMap.h>

using infobot_topo_msgs::TopologicalMap;
using infobot_topo_msgs::TopologicalPlace;
using infobot_topo_msgs::TopologicalView;


class TopomapFilesServerNode
{
public:
  static const std::string DEFAULT_FRAME_ID;

public:
  TopomapFilesServerNode(int argc, char **argv);

  void spin();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle privateNh_;

  ros::ServiceServer listTopomapFilesService_;
  ros::ServiceServer publishTopomapService_;
  ros::ServiceServer getTopomapService_;
  ros::ServiceServer applyRectFilterService_;
  ros::Publisher topoMapPub_;

  std::string topoMapFilesDirStr_;

private:
  bool listTopoMapFilesCallback(infobot_topo_msgs::ListTopologicalMapFiles::Request  &req,
                                infobot_topo_msgs::ListTopologicalMapFiles::Response &res);
  bool publishTopoMapCallback(infobot_topo_msgs::PublishTopologicalMap::Request  &req,
                              infobot_topo_msgs::PublishTopologicalMap::Response &res);
  bool getTopoMapCallback(infobot_topo_msgs::GetTopologicalMap::Request  &req,
                          infobot_topo_msgs::GetTopologicalMap::Response &res);
  bool ApplyRectFilterCallback(infobot_topo_msgs::ApplyRectangleFilter::Request  &req,
                               infobot_topo_msgs::ApplyRectangleFilter::Response &res);

private:
  TopologicalMap lastTopoMap_;
  bool lastTopoMapSet_;
};

const std::string TopomapFilesServerNode::DEFAULT_FRAME_ID = "map";

void TopomapFilesServerNode::spin()
{
  ros::spin();
}

TopomapFilesServerNode::TopomapFilesServerNode(int argc, char **argv):
  nh_(),
  privateNh_("~"),
  lastTopoMapSet_(false)
{
  boost::filesystem::path cwd(boost::filesystem::current_path());
  privateNh_.param("topomap_files_dirpath", topoMapFilesDirStr_, cwd.string());  // by default cwd

  listTopomapFilesService_ = nh_.advertiseService(
                               "list_topomap_files", &TopomapFilesServerNode::listTopoMapFilesCallback, this);
  publishTopomapService_ = nh_.advertiseService(
                             "publish_topomap", &TopomapFilesServerNode::publishTopoMapCallback, this);
  getTopomapService_ = nh_.advertiseService(
                         "get_topomap", &TopomapFilesServerNode::getTopoMapCallback, this);
  applyRectFilterService_ = nh_.advertiseService(
                         "apply_rect_filter", &TopomapFilesServerNode::ApplyRectFilterCallback, this);

  topoMapPub_ = nh_.advertise<TopologicalMap>("topomap", 1,  true);
}

bool TopomapFilesServerNode::listTopoMapFilesCallback(infobot_topo_msgs::ListTopologicalMapFiles::Request &req,
    infobot_topo_msgs::ListTopologicalMapFiles::Response &res)
{
  namespace fs = ::boost::filesystem;

  fs::path root(fs::canonical(topoMapFilesDirStr_));
  if (!fs::exists(root) || !fs::is_directory(root)) return false;

  fs::recursive_directory_iterator it(root);
  fs::recursive_directory_iterator endit;

  while (it != endit)
  {
    if (fs::is_regular_file(*it) && (it->path().extension() == ".yaml"))
    {
      // std::cout << it->path().filename() << std::endl;
      res.filenames.push_back(it->path().filename().string());
    }
    ++it;
  }
  return true;
}

bool TopomapFilesServerNode::publishTopoMapCallback(infobot_topo_msgs::PublishTopologicalMap::Request &req,
    infobot_topo_msgs::PublishTopologicalMap::Response &res)
{
  namespace fs = ::boost::filesystem;

  fs::path filepath(fs::canonical(topoMapFilesDirStr_));
  filepath /= req.filename;

  std::string frame_id = "";
  if (req.frame_id == "")
  {
    ROS_WARN("frame_id field cannot be empty. Setting it to \"map\".");
    frame_id = DEFAULT_FRAME_ID;
  }

  // from infobot_topo_server/src/topo_server.cpp
  ROS_INFO("Loading topological map from file '%s'",  filepath.string().c_str());
  TopologicalMap topo_map;

  // Load from file
  YAML::Node main_node;
  try
  {
    main_node = YAML::LoadFile(filepath.string());
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("Error while loading yaml file: %s", filepath.string().c_str());
    return false;
  }
  topo_map.topo_map_id = main_node["id"].as<std::string>();
  topo_map.header.seq = 1;
  topo_map.header.stamp = ros::Time::now();
  topo_map.header.frame_id = frame_id;  // This just helps when displaying things in RViz

  // Places
  YAML::Node places_node =  main_node["places"];
  if (places_node.size() == 0)
  {
    ROS_ERROR("No places found in the topo map!");
    return false;
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
      return false;
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
  topoMapPub_.publish(topo_map);
  ROS_INFO("Topological map with ID: %s and %d places published!",
           topo_map.topo_map_id.c_str(),
           topo_map.places.size());
  lastTopoMap_ = topo_map;
  lastTopoMapSet_ = true;

  res.success = true;
  return true;
}

bool TopomapFilesServerNode::getTopoMapCallback(infobot_topo_msgs::GetTopologicalMap::Request &req,
    infobot_topo_msgs::GetTopologicalMap::Response &res)
{
  namespace fs = ::boost::filesystem;

  std::string frame_id = "";
  if (req.frame_id == "")
  {
    ROS_WARN("frame_id field cannot be empty. Setting it to \"map\".");
    frame_id = DEFAULT_FRAME_ID;
  }

  fs::path filepath(fs::canonical(topoMapFilesDirStr_));
  if (req.filename == "" && !lastTopoMapSet_)
  {
    ROS_ERROR("filename field is empty, but no topomap has been published.");
    return false;
  }
  else if (req.filename == "" && lastTopoMapSet_)
  {
    ROS_WARN("filename field is empty, will return the most recently published topomap.");
    ROS_WARN("Overriding old frame_id with new frame_id \"%s\".", frame_id.c_str());
    // Response
    res.topomap = lastTopoMap_;
    return true;
  }
  else
    filepath /= req.filename;

  // from infobot_topo_server/src/topo_server.cpp
  ROS_INFO("Loading topological map from file '%s'",  filepath.string().c_str());
  TopologicalMap topo_map;

  // Load from file
  YAML::Node main_node;
  try
  {
    main_node = YAML::LoadFile(filepath.string());
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("Error while loading yaml file: %s", filepath.string().c_str());
    return false;
  }
  topo_map.topo_map_id = main_node["id"].as<std::string>();
  topo_map.header.seq = 1;
  topo_map.header.stamp = ros::Time::now();
  topo_map.header.frame_id = frame_id;  // This just helps when displaying things in RViz

  // Places
  YAML::Node places_node =  main_node["places"];
  if (places_node.size() == 0)
  {
    ROS_ERROR("No places found in the topo map!");
    return false;
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
      return false;
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

  // Response
  res.topomap = topo_map;
  ROS_INFO("Returned a topological map with ID: %s and %d places",
           topo_map.topo_map_id.c_str(),
           topo_map.places.size());
  return true;
}

bool TopomapFilesServerNode::ApplyRectFilterCallback(
  infobot_topo_msgs::ApplyRectangleFilter::Request &req,
  infobot_topo_msgs::ApplyRectangleFilter::Response &res)
{
  if (!lastTopoMapSet_)
  {
    ROS_ERROR("No topomap has been published. Cannot apply filter.");
    return false;
  }
  if ((req.min_x >= req.max_x) || (req.min_y >= req.max_y))
  {
    ROS_ERROR("Invalid input min_x=%f, min_y=%f, max_x=%f, max_y=%f.",
              req.min_x, req.min_y, req.max_x, req.max_y);
    return false;
  }

  TopologicalMap topomap;
  for (int i = 0; i < lastTopoMap_.places.size(); ++i)
  {
     if (lastTopoMap_.places[i].views.size() == 0)
       continue;
     double x = lastTopoMap_.places[i].views[0].pose.x;
     double y = lastTopoMap_.places[i].views[0].pose.y;
     if (x > req.min_x && x < req.max_x && y > req.min_y && y < req.max_y)
       topomap.places.push_back(lastTopoMap_.places[i]);
  }
  topomap.topo_map_id = lastTopoMap_.topo_map_id;
  topomap.header.seq = 1;
  topomap.header.stamp = ros::Time::now();
  topomap.header.frame_id = lastTopoMap_.header.frame_id;
  lastTopoMap_ = topomap;
  res.success = true;
  return true;
}

int main(int argc, char** argv)
{
  // ROS initialization.
  ros::init(argc, argv, "octomap_files_server");

  TopomapFilesServerNode node(argc, argv);
  node.spin();

  return 0;
}
