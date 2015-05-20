/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: octomap_from_map_static_server <map.yaml> [windows.yaml]\n" \
              "  map.yaml: map description file\n" \
              "  windows.yaml: (optional) window locations description file\n"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <map_server/image_loader.h>
#include <nav_msgs/MapMetaData.h>
#include <yaml-cpp/yaml.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

// From: http://stackoverflow.com/questions/11716268/point-in-polygon-algorithm
inline int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if (((verty[i] > testy) != (verty[j] > testy)) &&
        (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]))
       c = !c;
  }
  return c;
}

inline void loadYAML(const std::string &filename, YAML::Node &doc)
{
  std::ifstream fin(filename.c_str());
  if (fin.fail())
  {
    ROS_ERROR("octomap_from_map_static_server could not open %s.", filename.c_str());
    exit(-1);
  }

#ifdef HAVE_NEW_YAMLCPP
  // The document loading process changed in yaml-cpp 0.5.
  doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  // YAML::Node doc;
  parser.GetNextDocument(doc);
#endif
}

class OctomapFromMapStaticServer
{
public:
  /** Trivial constructor */
  OctomapFromMapStaticServer(const std::string& fname, const std::string& winfname)
  {
    std::string mapfname = "";
    double origin[3];
    int negate;
    double occ_th, free_th;
    bool trinary = true;
    std::string frame_id;
    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", frame_id, std::string("map"));

    double res = 0.05;
    double probHit = 0.0;
    double probMiss = 0.0;
    double thresMin = 0.0;
    double thresMax = 0.0;
    double wallHeight = 0.0;
    double windowHeightBeg = 0.85;
    double windowHeightEnd = 0.0;

    // Default values are the right most arguments.
    private_nh.param("resolution", res, 0.05);
    private_nh.param("sensor_model/hit", probHit, 0.7);
    private_nh.param("sensor_model/miss", probMiss, 0.4);
    private_nh.param("sensor_model/min", thresMin, 0.12);
    private_nh.param("sensor_model/max", thresMax, 0.97);
    private_nh.param("wall_height", wallHeight, 2.0);
    private_nh.param("window_height_beg", windowHeightBeg, 0.85);
    private_nh.param("window_height_end", windowHeightEnd, 2.35);

    // Parse Map YAML file.
    YAML::Node doc;
    loadYAML(fname, doc);
    try
    {
      doc["resolution"] >> res;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["negate"] >> negate;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain a negate tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["occupied_thresh"] >> occ_th;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["free_thresh"] >> free_th;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["trinary"] >> trinary;
    }
    catch (YAML::Exception)
    {
      ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
      trinary = true;
    }
    try
    {
      doc["origin"][0] >> origin[0];
      doc["origin"][1] >> origin[1];
      doc["origin"][2] >> origin[2];
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain an origin tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["image"] >> mapfname;
      if (mapfname.size() == 0)
      {
        ROS_ERROR("The image tag cannot be an empty string.");
        exit(-1);
      }
      if (mapfname[0] != '/')
      {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
      }
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain an image tag or it is invalid.");
      exit(-1);
    }

    ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
    map_server::loadMapFromFile(&map_resp_, mapfname.c_str(), res, negate, occ_th, free_th, origin, trinary);
    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = frame_id;
    map_resp_.map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
             map_resp_.map.info.width,
             map_resp_.map.info.height,
             map_resp_.map.info.resolution);
    meta_data_message_ = map_resp_.map.info;

    // Parse Window YAML File.
    std::vector<std::vector <float>> winPolyXs;
    std::vector<std::vector <float>> winPolyYs;
    if (winfname != "") {
      YAML::Node windoc;
      loadYAML(winfname, windoc);
      try
      {
        if (windoc["windows"].size() == 0)
        {
          ROS_ERROR("No windows found in the file!");
          exit(-1);
        }
        double xTmp, yTmp;
        for (int i = 0; i < windoc["windows"].size(); i++)
        {
          YAML::Node coorddoc = windoc["windows"][i]["coordinates"];
          std::vector <float> winPolyXTmp;
          std::vector <float> winPolyYTmp;
          for (int j = 0; j < coorddoc.size(); ++j)
          {
            coorddoc[j][0] >> xTmp;
            coorddoc[j][1] >> yTmp;
            winPolyXTmp.push_back(xTmp);
            winPolyYTmp.push_back(yTmp);
          }
          winPolyXs.push_back(winPolyXTmp);
          winPolyYs.push_back(winPolyYTmp);
        }
      }
      catch (YAML::InvalidScalar)
      {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
      }
    }

    // Start ROS Components.
    service = n.advertiseService("octomap_binary", &OctomapFromMapStaticServer::mapCallback, this);
    pub = n.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);


    // Create octree.
    octomap::OcTree* octree = NULL;

    octree = new octomap::OcTree(res);
    octree->setProbHit(probHit);
    octree->setProbMiss(probMiss);
    octree->setClampingThresMin(thresMin);
    octree->setClampingThresMax(thresMax);

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree->getMetricMin(minX, minY, minZ);
    octree->getMetricMax(maxX, maxY, maxZ);

    // building octree from OccupancyGrid
    if (map_resp_.map.info.width * map_resp_.map.info.height == 0)
    {
      std::stringstream ss;
      ss << "Map is zero-sized (" << map_resp_.map.info.width << "x" << map_resp_.map.info.height << ")";
      ROS_ERROR(ss.str().c_str());
      return;
    }

    ROS_DEBUG("Previously received a %d X %d map @ %.3f m/pix\n",
              map_resp_.map.info.width,
              map_resp_.map.info.height,
              map_resp_.map.info.resolution);

    double originPosX = map_resp_.map.info.origin.position.x;
    double originPosY = map_resp_.map.info.origin.position.y;
    double resolution = map_resp_.map.info.resolution;
    int width = map_resp_.map.info.width;
    int height = map_resp_.map.info.height;

    // map and octomap has different "center points".
    octomap::KeySet occupiedCells;
    double delta = resolution / 2.0;  // octomap's center is in middle of square.
    for (int h = 0; h < height; ++h)
    {
      for (int w = 0; w < width; ++w)
      {
        double x = originPosX + w * resolution + delta;
        double y = originPosY + h * resolution + delta;

        int v = map_resp_.map.data[h * width + w];
        // only support trinary mode, allowing only 100, 0, -1
        if (v != 100 && v != 0 && v != -1)
        {
          ROS_ERROR("Map is NOT using the trinary mode. Unsupported format.");
          delete octree;
          return;
        }

        if (v == 100)  // occupied
        {
          // octomap::point3d pBeg(x, y, resolution / 2.0);
          // octomap::point3d pEnd(x, y, resolution / 2.0 + wallHeight_);
          // octomap::KeyRay keyRay;
          // if (octree->computeRayKeys(pBeg, pEnd, keyRay))
          //   occupiedCells.insert(keyRay.begin(), keyRay.end());
          bool isWindow = false;
          for (int i = 0; i < winPolyXs.size(); ++i)
          {
            if (pnpoly(winPolyXs[i].size(), &winPolyXs[i][0], &winPolyYs[i][0], x, y))
              isWindow = true;
          }
          for (double z = 0; z < wallHeight; z+=resolution)
          {
            if (isWindow && z > windowHeightBeg && z < windowHeightEnd)
              continue;

            octomap::point3d pt(x, y, z + (resolution / 2.0));
            octomap::OcTreeKey key;
            if (octree->coordToKeyChecked(pt, key))
              occupiedCells.insert(key);
            else
              ROS_ERROR_STREAM("Could not generate Key for point " << pt);
          }
        }
      }
    }

    for (octomap::KeySet::iterator it = occupiedCells.begin(), end = occupiedCells.end(); it != end; it++)
      octree->updateNode(*it, true);

    // Publish and save results
    lastMap_.header.frame_id = frame_id;
    lastMap_.header.stamp = ros::Time::now();
    bool publishMsg = octomap_msgs::binaryMapToMsg(*octree, lastMap_);
    delete octree;

    if (publishMsg)
    {
      lastMapAvailable_ = true;
      pub.publish(lastMap_);
    }
    else
    {
      lastMapAvailable_ = false;
      ROS_ERROR("Error serializing OctoMap");
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::ServiceServer service;
  octomap_msgs::Octomap lastMap_;
  bool lastMapAvailable_;

  /** Callback invoked when someone requests our service */
  bool mapCallback(octomap_msgs::GetOctomap::Request  &req,
                   octomap_msgs::GetOctomap::Response &res)
  {
    if (!lastMapAvailable_)
    return false;

    res.map = lastMap_;
    return true;
  }

  /** The map data is cached here, to be sent out to service callers
   */
  nav_msgs::MapMetaData meta_data_message_;
  nav_msgs::GetMap::Response map_resp_;

  /*
  void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
  {
    pub.publish( meta_data_message_ );
  }
  */
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_from_map_static_server", ros::init_options::AnonymousName);
  if (argc != 2 && argc != 3)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  std::string fname(argv[1]);
  std::string winfname("");
  if (argc == 3)
    winfname.append(argv[2]);

  try
  {
    OctomapFromMapStaticServer ms(fname, winfname);
    ros::spin();
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

