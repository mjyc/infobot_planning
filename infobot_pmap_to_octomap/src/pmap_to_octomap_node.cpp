#include <string>
#include <vector>
#include <limits>
#include <random>
#include <map>

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <infobot_mapping_msgs/ProbabilityGrid.h>
#include <infobot_octomap_store/GetOctomap.h>
#include <infobot_pmap_to_octomap/GetProbabilityOctomapSurface.h>
#include <infobot_pmap_to_octomap/GetProbabilityOctomapHeight.h>
#include <infobot_pmap_store/GetProbabilityMap.h>

#include "./validate_float.h"


class ProbabilityMapToOctomapNode
{
public:
  static const double DEFAULT_GROUND_HEIGHT;
  static const double MAX_SURFACE_HEIGHT;
  static const std::string DEFAULT_FRAME_ID;

public:
  ProbabilityMapToOctomapNode(int argc, char **argv);

  void spin();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle privateNh_;

  ros::Publisher pub_;
  ros::ServiceServer surfaceService_;
  ros::ServiceServer heightService_;

private:
  bool checkRequestArgs(const std::string &pmap_filename,
                        const std::string &octomap_filename,
                        std::string &pmap_frame_id,
                        std::string &octomap_frame_id);
  bool getProbabilityMap(const std::string &pmap_filename,
                        const std::string &pmap_frame_id,
                        infobot_mapping_msgs::ProbabilityGrid &pmap);
  bool getOctomap(const std::string &octomap_filename,
                  const std::string &octomap_frame_id,
                  octomap_msgs::Octomap &octomap);

  bool surfaceCallback(infobot_pmap_to_octomap::GetProbabilityOctomapSurface::Request  &req,
                       infobot_pmap_to_octomap::GetProbabilityOctomapSurface::Response &res);
  bool heightCallback(infobot_pmap_to_octomap::GetProbabilityOctomapHeight::Request  &req,
                      infobot_pmap_to_octomap::GetProbabilityOctomapHeight::Response &res);

  bool applyProbabilityMapToOctomapSurface(const infobot_mapping_msgs::ProbabilityGrid &pmap,
                                           int probValOffset,
                                           octomap::OcTree* octree);
  bool applyProbabilityMapToOctomapHeight(const infobot_mapping_msgs::ProbabilityGrid &pmap,
                                          int probValOffset,
                                          double heightMean,
                                          double heightVar,
                                          octomap::OcTree* octree);
};

const double ProbabilityMapToOctomapNode::DEFAULT_GROUND_HEIGHT = 0.0;  // C++11 style
const double ProbabilityMapToOctomapNode::MAX_SURFACE_HEIGHT = 1.5;
const std::string ProbabilityMapToOctomapNode::DEFAULT_FRAME_ID = "map";

//----------------------------------------------------------------------
// Public Methods
//----------------------------------------------------------------------

ProbabilityMapToOctomapNode::ProbabilityMapToOctomapNode(int argc, char **argv):
  nh_(),
  privateNh_("~")
{
  pub_ = nh_.advertise<octomap_msgs::Octomap>("poctomap", 1, true);
  surfaceService_ = nh_.advertiseService("get_poctomap_surface", &ProbabilityMapToOctomapNode::surfaceCallback, this);
  heightService_ = nh_.advertiseService("get_poctomap_height", &ProbabilityMapToOctomapNode::heightCallback, this);
}

void ProbabilityMapToOctomapNode::spin()
{
  ros::spin();
}

//----------------------------------------------------------------------
// Private Helper Methods
//----------------------------------------------------------------------

bool ProbabilityMapToOctomapNode::checkRequestArgs(const std::string &pmap_filename,
                                                   const std::string &octomap_filename,
                                                   std::string &pmap_frame_id,
                                                   std::string &octomap_frame_id)
{
  // frame_id can be modified
  if (pmap_frame_id.empty())
  {
    ROS_WARN("pmap_frame_id field cannot be empty. Setting it to \"map\".");
    privateNh_.param("frame_id", pmap_frame_id, DEFAULT_FRAME_ID);
  }
  if (octomap_frame_id.empty())
  {
    ROS_WARN("octomap_frame_id field cannot be empty. Setting it to \"map\".");
    privateNh_.param("frame_id", octomap_frame_id, DEFAULT_FRAME_ID);
  }
  if (pmap_frame_id != octomap_frame_id)
  {
    ROS_WARN("pmap_frame_id and octomap_frame_id must be same.");
    return false;
  }
  return true;
}

bool ProbabilityMapToOctomapNode::getProbabilityMap(const std::string &pmap_filename,
                                                    const std::string &pmap_frame_id,
                                                    infobot_mapping_msgs::ProbabilityGrid &pmap)
{
  ROS_INFO("Waiting for /get_pmap service ...");
  ros::service::waitForService("/get_pmap");
  ROS_INFO("Waiting for /get_pmap service Done!");
  ros::ServiceClient getProbabilityMapClient = nh_.serviceClient<infobot_pmap_store::GetProbabilityMap>("/get_pmap");
  infobot_pmap_store::GetProbabilityMap getProbabilityMapSrv;
  getProbabilityMapSrv.request.filename = pmap_filename;
  getProbabilityMapSrv.request.frame_id = pmap_frame_id;
  if (getProbabilityMapClient.call(getProbabilityMapSrv))
  {
    pmap = getProbabilityMapSrv.response.pmap;
  }
  else
  {
    ROS_ERROR("Failed to call service /get_pmap with filename=%s and frame_id=%s",
      pmap_filename.c_str(), pmap_frame_id.c_str());
    return false;
  }
}

bool ProbabilityMapToOctomapNode::getOctomap(const std::string &octomap_filename,
                                             const std::string &octomap_frame_id,
                                             octomap_msgs::Octomap &octomap)
{
  ROS_INFO("Waiting for /get_octomap service ...");
  ros::service::waitForService("/get_octomap");
  ROS_INFO("Waiting for /get_octomap service Done!");
  ros::ServiceClient getOctomapClient = nh_.serviceClient<infobot_octomap_store::GetOctomap>("/get_octomap");
  infobot_octomap_store::GetOctomap getOctomapSrv;
  getOctomapSrv.request.filename = octomap_filename;
  getOctomapSrv.request.frame_id = octomap_frame_id;
  if (getOctomapClient.call(getOctomapSrv))
  {
    octomap = getOctomapSrv.response.octomap;
  }
  else
  {
    ROS_ERROR("Failed to call service /octomap_binary");
    return false;
  }

  // We only make use of octomap binary
  if (!octomap.binary)
  {
    ROS_ERROR("Received octomap's format is not binary.");
    return false;
  }
}

//----------------------------------------------------------------------
// Private Service Callback Methods
//----------------------------------------------------------------------

bool ProbabilityMapToOctomapNode::surfaceCallback(
  infobot_pmap_to_octomap::GetProbabilityOctomapSurface::Request  &req,
  infobot_pmap_to_octomap::GetProbabilityOctomapSurface::Response &res)
{
  std::string pmap_filename = req.pmap_filename;
  std::string pmap_frame_id = req.pmap_frame_id;
  std::string octomap_filename = req.octomap_filename;
  std::string octomap_frame_id = req.octomap_frame_id;
  if (!checkRequestArgs(pmap_filename, octomap_filename, pmap_frame_id, octomap_frame_id))
    return false;

  infobot_mapping_msgs::ProbabilityGrid pmap;
  if (!getProbabilityMap(pmap_filename, pmap_frame_id, pmap))
    return false;

  octomap_msgs::Octomap octomap;
  if (!getOctomap(octomap_filename, octomap_frame_id, octomap))
    return false;

  octomap::OcTree* octree = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap);
  if (tree)
  {
    octree = dynamic_cast<octomap::OcTree*>(tree);
  }

  int probValOffset = static_cast<int>(octree->getClampingThresMaxLog() + 0.5);
  ROS_DEBUG("probability_value_offset=%d", probValOffset);
  if (!applyProbabilityMapToOctomapSurface(pmap, probValOffset, octree))
  {
    delete octree;
    return false;
  }

  // Publish and save results
  octomap_msgs::Octomap map;
  map.header.frame_id = octomap.header.frame_id;
  map.header.stamp = ros::Time::now();
  if (octomap_msgs::fullMapToMsg(*octree, map))
  {
    res.octomap = map;
    pub_.publish(map);
  }
  else
    ROS_ERROR("Error serializing OctoMap");

  // Clean up
  delete octree;

  return true;
}

bool ProbabilityMapToOctomapNode::heightCallback(
  infobot_pmap_to_octomap::GetProbabilityOctomapHeight::Request  &req,
  infobot_pmap_to_octomap::GetProbabilityOctomapHeight::Response &res)
{
  std::string pmap_filename = req.pmap_filename;
  std::string pmap_frame_id = req.pmap_frame_id;
  std::string octomap_filename = req.octomap_filename;
  std::string octomap_frame_id = req.octomap_frame_id;
  if (!checkRequestArgs(pmap_filename, octomap_filename, pmap_frame_id, octomap_frame_id))
    return false;

  infobot_mapping_msgs::ProbabilityGrid pmap;
  if (!getProbabilityMap(pmap_filename, pmap_frame_id, pmap))
    return false;

  octomap_msgs::Octomap octomap;
  if (!getOctomap(octomap_filename, octomap_frame_id, octomap))
    return false;

  octomap::OcTree* octree = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap);
  if (tree)
  {
    octree = dynamic_cast<octomap::OcTree*>(tree);
  }

  int probValOffset = static_cast<int>(octree->getClampingThresMaxLog() + 0.5);
  ROS_DEBUG("probability_value_offset=%d", probValOffset);
  if (!applyProbabilityMapToOctomapHeight(pmap, probValOffset, req.height_mean, req.height_var, octree))
  {
    delete octree;
    return false;
  }

  // Publish and save results
  octomap_msgs::Octomap map;
  map.header.frame_id = octomap.header.frame_id;
  map.header.stamp = ros::Time::now();
  if (octomap_msgs::fullMapToMsg(*octree, map))
  {
    res.octomap = map;
    pub_.publish(map);
  }
  else
    ROS_ERROR("Error serializing OctoMap");

  // Clean up
  delete octree;

  return true;
}

//----------------------------------------------------------------------
// Private Service Callback Helper Methods
//----------------------------------------------------------------------

bool ProbabilityMapToOctomapNode::applyProbabilityMapToOctomapSurface(
  const infobot_mapping_msgs::ProbabilityGrid &pmap, int probValOffset, octomap::OcTree* octree)
{
  if (!validateFloats(pmap))
  {
    ROS_ERROR("ProbabilityGrid contained invalid floating point values (nans or infs)");
    return false;
  }
  if (pmap.info.width * pmap.info.height == 0)
  {
    std::stringstream ss;
    ss << "ProbabilityGrid is zero-sized (" << pmap.info.width << "x" << pmap.info.height << ")";
    ROS_ERROR(ss.str().c_str());
    return false;
  }
  if (pmap.info.origin.orientation.x != 0.0 || pmap.info.origin.orientation.y != 0.0 ||
      pmap.info.origin.orientation.z != 0.0 || pmap.info.origin.orientation.w != 1.0)
  {
    ROS_ERROR("Currently does not support input ProbabilityGrid with non-zero rotation.");
    return false;
  }

  float origX = pmap.info.origin.position.x;
  float origY = pmap.info.origin.position.y;
  float res = pmap.info.resolution;
  int width = pmap.info.width;
  int height = pmap.info.height;

  double minX, minY, minZ, maxX, maxY, maxZ;
  octree->getMetricMin(minX, minY, minZ);
  octree->getMetricMax(maxX, maxY, maxZ);
  ROS_DEBUG("BBX Min Coord: %d %d %d / BBX Max Coord: %d %d %d", minX, minY, minZ, maxX, maxY, maxZ);
  for (int h = 0; h < height; ++h)
  {
    for (int w = 0; w < width; ++w)
    {
      double curProb = pmap.data[h * width + w];
      double x = origX + (w * res) + (res / 2.0);
      double y = origY + (h * res) + (res / 2.0);
      if (curProb > 0.0 && x > minX && x < maxX && y > minY && y < maxY)
      {
        octomap::point3d minBBXPt(x, y, minZ);
        octomap::point3d maxBBXPt(x, y, maxZ);
        double z = minZ;  // surface z
        for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(minBBXPt, maxBBXPt),
             end = octree->end_leafs_bbx(); it != end; ++it)
        {
          if (octree->isNodeOccupied(*it))
          {
            if (it.getZ() > MAX_SURFACE_HEIGHT)  // do not consider > max
              continue;
            if (z < it.getZ())
              z = it.getZ();
          }
        }

        octomap::point3d pt(x, y, z);
        octomap::OcTreeKey key;
        if (octree->coordToKeyChecked(pt, key))
        {
          if (z != minZ && octree->search(key) != 0)
          {
            double pval = 1.0;
            if (octree->search(key)->getValue() >= probValOffset)
              pval = octree->search(key)->getValue() - probValOffset;
            else
              octree->updateNode(key, true);
            octree->search(key)->setValue(probValOffset + (curProb * pval));
          }
          else  // if the point is in unmapped area
          {
            z = DEFAULT_GROUND_HEIGHT;
            octomap::point3d pt_ground(x, y, z);
            octomap::OcTreeKey key_ground;
            if (octree->coordToKeyChecked(pt_ground, key_ground))
            {
              octree->updateNode(key_ground, true);
              octree->search(key_ground)->setValue(probValOffset + (curProb));
            }
          }
        }
      }
    }
  }

  // Set occupied, but unused node's value to "probValOffset" (meaning probability = 0.0)
  for (octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it)
  {
    if (octree->isNodeOccupied(*it) && it->getValue() < probValOffset)
      it->setValue(probValOffset);
  }

  return true;
}

bool ProbabilityMapToOctomapNode::applyProbabilityMapToOctomapHeight(
  const infobot_mapping_msgs::ProbabilityGrid &pmap, int probValOffset, double heightMean, double heightVar,
  octomap::OcTree* octree)
{
  if (!validateFloats(pmap))
  {
    ROS_ERROR("ProbabilityGrid contained invalid floating point values (nans or infs)");
    return false;
  }
  if (pmap.info.width * pmap.info.height == 0)
  {
    std::stringstream ss;
    ss << "ProbabilityGrid is zero-sized (" << pmap.info.width << "x" << pmap.info.height
       << ")";
    ROS_ERROR(ss.str().c_str());
    return false;
  }
  if (pmap.info.origin.orientation.x != 0.0 || pmap.info.origin.orientation.y != 0.0 ||
      pmap.info.origin.orientation.z != 0.0 || pmap.info.origin.orientation.w != 1.0)
  {
    ROS_ERROR("Currently does not support input ProbabilityGrid with non-zero rotation.");
    return false;
  }

  float origX = pmap.info.origin.position.x;
  float origY = pmap.info.origin.position.y;
  float res = pmap.info.resolution;
  int width = pmap.info.width;
  int height = pmap.info.height;


  // Transfer data from ProbabilityGrid to Octomap.
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(heightMean, heightVar);

  double minX, minY, minZ, maxX, maxY, maxZ;
  octree->getMetricMin(minX, minY, minZ);
  octree->getMetricMax(maxX, maxY, maxZ);
  ROS_DEBUG("BBX Min Coord: %d %d %d / BBX Max Coord: %d %d %d", minX, minY, minZ, maxX, maxY, maxZ);
  for (int h = 0; h < height; ++h)
  {
    for (int w = 0; w < width; ++w)
    {
      double curProb = pmap.data[h * width + w];
      double x = origX + (w * res) + (res / 2.0);
      double y = origY + (h * res) + (res / 2.0);
      if (curProb > 0.0 && x > minX && x < maxX && y > minY && y < maxY)
      {
        double z = distribution(generator);
        octomap::point3d pt(x, y, z);
        octomap::OcTreeKey key;
        if (octree->coordToKeyChecked(pt, key))
        {
          octree->updateNode(key, true);
          octree->search(key)->setValue(probValOffset + (curProb));
        }
      }
    }
  }

  // Set occupied, but unused node's value to "probValOffset"
  for (octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it)
  {
    if (octree->isNodeOccupied(*it) && it->getValue() < probValOffset)
      it->setValue(probValOffset);
  }

  return true;
}

//----------------------------------------------------------------------
// Main
//----------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pmap_to_octomap");

  ProbabilityMapToOctomapNode node(argc, argv);
  node.spin();

  return 0;
}
