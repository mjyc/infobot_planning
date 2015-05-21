#include <string>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap.h>

#include <infobot_map_msgs/ListOctomapFiles.h>
#include <infobot_map_msgs/PublishOctomap.h>
#include <infobot_map_msgs/GetOctomap.h>


class OctomapFilesServerNode
{
public:
  static const std::string DEFAULT_FRAME_ID;

public:
  OctomapFilesServerNode(int argc, char **argv);

  void spin();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle privateNh_;
  boost::shared_ptr<octomap_server::OctomapServer> serverPtr_;

  ros::ServiceServer listOctomapFilesService_;
  ros::ServiceServer publishOctomapService_;
  ros::ServiceServer getOctomapService_;

  std::string octomapFilesDirStr_;

private:
  bool listOctomapFilesCallback(infobot_map_msgs::ListOctomapFiles::Request  &req,
                                infobot_map_msgs::ListOctomapFiles::Response &res);
  bool publishOctomapCallback(infobot_map_msgs::PublishOctomap::Request  &req,
                              infobot_map_msgs::PublishOctomap::Response &res);
  bool getOctomapCallback(infobot_map_msgs::GetOctomap::Request  &req,
                          infobot_map_msgs::GetOctomap::Response &res);
};

const std::string OctomapFilesServerNode::DEFAULT_FRAME_ID = "map";

void OctomapFilesServerNode::spin()
{
  ros::spin();
}

OctomapFilesServerNode::OctomapFilesServerNode(int argc, char **argv):
  nh_(),
  privateNh_("~"),
  serverPtr_()
{
  boost::filesystem::path cwd(boost::filesystem::current_path());
  privateNh_.param("octomap_files_dirpath", octomapFilesDirStr_, cwd.string());  // by default cwd

  listOctomapFilesService_ = nh_.advertiseService(
                               "list_octomap_files", &OctomapFilesServerNode::listOctomapFilesCallback, this);
  publishOctomapService_ = nh_.advertiseService(
                             "publish_octomap", &OctomapFilesServerNode::publishOctomapCallback, this);
  getOctomapService_ = nh_.advertiseService(
                         "get_octomap", &OctomapFilesServerNode::getOctomapCallback, this);
}

bool OctomapFilesServerNode::listOctomapFilesCallback(infobot_map_msgs::ListOctomapFiles::Request &req,
    infobot_map_msgs::ListOctomapFiles::Response &res)
{
  namespace fs = ::boost::filesystem;

  fs::path root(fs::canonical(octomapFilesDirStr_));
  if (!fs::exists(root) || !fs::is_directory(root)) return false;

  fs::recursive_directory_iterator it(root);
  fs::recursive_directory_iterator endit;

  while (it != endit)
  {
    if (fs::is_regular_file(*it) && (it->path().extension() == ".bt" || it->path().extension() == ".ot"))
    {
      // std::cout << it->path().filename() << std::endl;
      res.filenames.push_back(it->path().filename().string());
    }
    ++it;
  }
  return true;
}

bool OctomapFilesServerNode::publishOctomapCallback(infobot_map_msgs::PublishOctomap::Request &req,
    infobot_map_msgs::PublishOctomap::Response &res)
{
  namespace fs = ::boost::filesystem;

  fs::path filepath(fs::canonical(octomapFilesDirStr_));
  filepath /= req.filename;

  if (req.frame_id != "")
    privateNh_.setParam("frame_id", req.frame_id);
  else
    privateNh_.setParam("frame_id", DEFAULT_FRAME_ID);
  serverPtr_.reset();  // remove previous server object
  serverPtr_.reset(new octomap_server::OctomapServer(privateNh_));  // create new server object

  if (!serverPtr_->openFile(filepath.string()))
  {
    ROS_ERROR("Could not open file %s", filepath.string().c_str());
    serverPtr_.reset();  // delete octomap server
    return false;
  }
  res.success = true;
  return true;
}

bool OctomapFilesServerNode::getOctomapCallback(infobot_map_msgs::GetOctomap::Request &req,
    infobot_map_msgs::GetOctomap::Response &res)
{
  namespace fs = ::boost::filesystem;
  using octomap::OcTree;
  using octomap::AbstractOcTree;
  using octomap::AbstractOccupancyOcTree;

  if (req.frame_id != "")
    privateNh_.setParam("frame_id", req.frame_id);
  else
  {
    ROS_WARN("Empty input frame_id. Setting it to \"%s\".", DEFAULT_FRAME_ID.c_str());
    privateNh_.setParam("frame_id", DEFAULT_FRAME_ID);
  }

  std::string filename = req.filename;
  if (filename.length() == 0)
  {
    if (!serverPtr_)
    {
      ROS_ERROR("no pmap is published previously. use /publish_pmap to publish a map before use /get_map with an empty "
        "filename.");
      return false;
    }
    octomap_msgs::GetOctomap::Request reqTmp;
    octomap_msgs::GetOctomap::Response resTmp;
    if (!serverPtr_->octomapBinarySrv(reqTmp, resTmp))
      return false;
    res.octomap = resTmp.map;
    ROS_WARN("Overriding old frame_id with new frame_id.");
    privateNh_.param("frame_id", res.octomap.header.frame_id, DEFAULT_FRAME_ID);
    res.octomap.header.stamp = ros::Time::now();
    return true;
  }

  if (filename.length() < 3)
  {
    ROS_ERROR("Invalid filename=%s", filename.c_str());
    return false;
  }
  std::string suffix = filename.substr(filename.length() - 3, 3);

  fs::path filepath(fs::canonical(octomapFilesDirStr_));
  filepath /= filename;

  AbstractOccupancyOcTree* m_octree;
  bool is_binary;
  // .bt files only as OcTree, all other classes need to be in .ot files:
  if (suffix == ".bt")
  {
    OcTree* octree = new OcTree(filepath.string());
    is_binary = true;

    m_octree = octree;
  }
  else if (suffix == ".ot")
  {
    AbstractOcTree* tree = AbstractOcTree::read(filepath.string());
    if (!tree)
    {
      ROS_ERROR("Could not read octree from file");
      return false;
    }
    is_binary = false;

    m_octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
  }
  else
  {
    ROS_ERROR("Octree file does not have .bt or .ot extension");
    return false;
  }

  if (!m_octree)
  {
    ROS_ERROR("Could not read right octree class in file");
    return false;
  }

  ROS_INFO("Read octree type \"%s\" from file %s", m_octree->getTreeType().c_str(), filepath.string().c_str());
  ROS_INFO("Octree resultion: %f, size: %zu", m_octree->getResolution(), m_octree->size());

  privateNh_.param("frame_id", res.octomap.header.frame_id, DEFAULT_FRAME_ID);
  res.octomap.header.stamp = ros::Time::now();
  if (is_binary)
  {
    if (!octomap_msgs::binaryMapToMsg(*m_octree, res.octomap))
    {
      ROS_ERROR("Could not read octree from binary octomap file(.bt)");
      delete m_octree;
      return false;
    }
  }
  else
  {
    if (!octomap_msgs::fullMapToMsg(*m_octree, res.octomap))
    {
      ROS_ERROR("Could not read octree from full octomap file(.ot)");
      delete m_octree;
      return false;
    }
  }

  delete m_octree;
  return true;
}

int main(int argc, char** argv)
{
  // ROS initialization.
  ros::init(argc, argv, "octomap_files_server");

  OctomapFilesServerNode node(argc, argv);
  node.spin();

  return 0;
}
