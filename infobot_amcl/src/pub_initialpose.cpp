#include <string>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using std::cerr;
using std::endl;
using std::string;
using std::stringstream;

int main(int argc, char** argv)
{
  // Input check
  if (argc < 4)
  {
    cerr << "usage: " << argv[0] << " x y theta" << endl;
    exit(-1);
  }

  // Parse
  double x, y, theta;
  stringstream x_str(argv[1]);
  x_str >> x;
  stringstream y_str(argv[2]);
  y_str >> y;
  stringstream theta_str(argv[3]);
  theta_str >> theta;

  // Init ROS
  ros::init(argc, argv, "pub_initialpose");
  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

  // Create msg & Publish msg

  ros::Duration(1.0).sleep();

  std::string fixed_frame = "map";
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(quat,
                        pose.pose.pose.orientation);
  pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
  pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
  pose.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;
  ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
  pub.publish(pose);

  ros::Duration(1.0).sleep();

  return 0;
}
