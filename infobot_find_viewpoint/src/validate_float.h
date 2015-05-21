#ifndef INFOBOT_PMAP_TO_OCTOMAP_VALIDATE_FLOAT_H
#define INFOBOT_PMAP_TO_OCTOMAP_VALIDATE_FLOAT_H

#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <infobot_map_msgs/ProbabilityGrid.h>

inline bool validateFloats(float val)
{
  return !(isnan(val) || isinf(val));
}

inline bool validateFloats(double val)
{
  return !(isnan(val) || isinf(val));
}

inline bool validateFloats(const geometry_msgs::Point& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  return valid;
}

inline bool validateFloats(const geometry_msgs::Quaternion& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  valid = valid && validateFloats(msg.w);
  return valid;
}

inline bool validateFloats(const geometry_msgs::Pose& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.position);
  valid = valid && validateFloats(msg.orientation);
  return valid;
}

inline bool validateFloats(const infobot_map_msgs::ProbabilityGrid& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.info.resolution);
  valid = valid && validateFloats(msg.info.origin);
  return valid;
}

#endif // INFOBOT_PMAP_TO_OCTOMAP_VALIDATE_FLOAT_H
