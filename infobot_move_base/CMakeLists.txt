cmake_minimum_required(VERSION 2.8.3)
project(infobot_move_base)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs move_base base_local_planner clear_costmap_recovery navfn rotate_recovery dwa_local_planner costmap_2d smach smach_ros)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs move_base base_local_planner clear_costmap_recovery navfn rotate_recovery dwa_local_planner costmap_2d smach smach_ros
)
