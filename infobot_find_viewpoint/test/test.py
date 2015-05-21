import sys

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from infobot_find_viewpoint_msgs.srv import *
from infobot_map_msgs.srv import *


def compute_vis_value_client(mode, camera_pose, pmap_filename, pmap_frame_id, octomap_filename, octomap_frame_id):
    rospy.wait_for_service("compute_vis_value")
    try:
        compute_vis_value = rospy.ServiceProxy("compute_vis_value", ComputeVisibilityValue)
        resp1 = compute_vis_value(mode, camera_pose, pmap_filename, pmap_frame_id, octomap_filename, octomap_frame_id)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":
    rospy.wait_for_service("publish_pmap")
    publish_pmap = rospy.ServiceProxy("publish_pmap", PublishProbabilityMap)
    # if not publish_pmap("real-floor4_491_people.yaml", ""):
    if not publish_pmap("CSE491.yaml", ""):
        sys.exit(1)

    rospy.wait_for_service("publish_octomap")
    publish_octomap = rospy.ServiceProxy("publish_octomap", PublishOctomap)
    # if not publish_octomap("rselab.bt", ""):
    # if not publish_octomap("real-floor4_skeleton.bt", ""):
    if not publish_octomap("real-floor4_corridor-model.bt", ""):
        sys.exit(1)

    pose = Pose()
    pose.position.x = 7.76184
    pose.position.y = 17.4192
    pose.position.z = 1.5
    q = quaternion_from_euler(0, 0, -2.83898)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    print compute_vis_value_client(0, pose, "", "", "", "")
