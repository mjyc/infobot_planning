import sys

import rospy
from infobot_find_viewpoint_msgs.srv import *
from infobot_map_msgs.srv import *
from infobot_topo_msgs.srv import *

import actionlib
from move_base_msgs.msg import *


def compute_vis_values_client(mode, pmap_filename, pmap_frame_id, octomap_filename, octomap_frame_id,
                              topomap_filename, topomap_frame_id):
    rospy.wait_for_service("compute_vis_values")
    try:
        compute_vis_values = rospy.ServiceProxy("compute_vis_values", ComputeVisibilityValues)
        resp1 = compute_vis_values(mode, pmap_filename, pmap_frame_id, octomap_filename, octomap_frame_id,
                                   topomap_filename, topomap_frame_id)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print >> sys.stderr, "test_action.py <pmap_filename>"
        sys.exit(1)

    rospy.init_node("visibility_action")

    rospy.wait_for_service("publish_pmap")
    publish_pmap = rospy.ServiceProxy("publish_pmap", PublishProbabilityMap)
    publish_pmap_resp = publish_pmap(sys.argv[1], "")
    if not publish_pmap_resp or not publish_pmap_resp.success:
        sys.exit(1)

    rospy.wait_for_service("publish_octomap")
    publish_octomap = rospy.ServiceProxy("publish_octomap", PublishOctomap)
    publish_octomap_resp = publish_octomap("real-floor4_corridor-model.bt", "")
    if not publish_octomap_resp or not publish_octomap_resp.success:
        sys.exit(1)

    rospy.wait_for_service("publish_topomap")
    publish_topomap = rospy.ServiceProxy("publish_topomap", PublishTopologicalMap)
    publish_topomap_resp = publish_topomap("real-floor4_corridor-nogo-topo.yaml", "")
    if not publish_topomap_resp or not publish_topomap_resp.success:
        sys.exit(1)

    # Get current pmap, compute bbx, apply filter to topomap
    rospy.wait_for_service("get_pmap")
    get_pmap = rospy.ServiceProxy("get_pmap", GetProbabilityMap)
    get_pmap_resp = get_pmap("", "")
    if not get_pmap_resp:
        sys.exit(1)

    margin = 5.0  # in meters
    minX = get_pmap_resp.pmap.info.origin.position.x - margin
    maxX = get_pmap_resp.pmap.info.origin.position.x + \
        (get_pmap_resp.pmap.info.width * get_pmap_resp.pmap.info.resolution) + margin
    minY = get_pmap_resp.pmap.info.origin.position.y - margin
    maxY = get_pmap_resp.pmap.info.origin.position.y + \
        (get_pmap_resp.pmap.info.height * get_pmap_resp.pmap.info.resolution) + margin

    print "applying filter to region:", minX, maxX, minY, maxY

    rospy.wait_for_service("apply_rect_filter")
    apply_rect_filter = rospy.ServiceProxy("apply_rect_filter", ApplyRectangleFilter)
    if not apply_rect_filter(minX, minY, maxX, maxY):
        sys.exit(1)

    resp = compute_vis_values_client(1, "", "", "", "", "", "")
    print "cur max i=", resp.max_vis_value_index
    prev_val = resp.max_vis_value_index


    action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    action_client.wait_for_server()
    for i in range(10):
        print "sending move base goal"
        cur_pose = resp.poses[resp.max_vis_value_index]
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = cur_pose.position.x
        goal.target_pose.pose.position.y = cur_pose.position.y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = cur_pose.orientation
        action_client.send_goal(goal)
        action_client.wait_for_result()

        print "recomputing vis_values"
        resp = compute_vis_values_client(0, "", "", "", "", "", "")
        print "cur max i=", resp.max_vis_value_index
        if (prev_val == resp.max_vis_value_index):
            print "Done!"
            break
        else:
            prev_val = resp.max_vis_value_index
