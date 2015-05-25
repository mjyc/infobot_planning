#!/usr/bin/env python

import rospy

import sys

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from infobot_find_viewpoint_msgs.srv import *
from infobot_map_msgs.srv import *
from infobot_topo_msgs.srv import *

PROSILICA_TRANS_Z = 1.314


def call_srv(srv_name, srv_type, srv_args):
    rospy.wait_for_service(srv_name)
    try:
        srv_proxy = rospy.ServiceProxy(srv_name, srv_type)
        return srv_proxy(*srv_args)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return None


def publish_pmap_octomap(pmap_filename, octomap_fiilename):
    publish_pmap_resp = call_srv("publish_pmap", PublishProbabilityMap, (pmap_filename, ""))
    if not publish_pmap_resp or not publish_pmap_resp.success:
        sys.exit(1)
    publish_octomap_resp = call_srv("publish_octomap", PublishOctomap, (octomap_fiilename, ""))
    if not publish_octomap_resp or not publish_octomap_resp.success:
        sys.exit(1)


def publish_pmap_octomap_topomap(pmap_filename, octomap_fiilename, topomap_filename):
    publish_pmap_octomap(pmap_filename, octomap_fiilename)
    publish_topomap_resp = call_srv("publish_topomap", PublishTopologicalMap, (topomap_filename, ""))
    if not publish_topomap_resp or not publish_topomap_resp.success:
        sys.exit(1)


def call_compute_vis_value_srv(pose, mode):
    resp = call_srv("compute_vis_value", ComputeVisibilityValue, (
        pose,
        mode,
        "",  # use published pmap_filaname
        "",  # use published pmap_frame
        "",  # use published octomap_filaname
        ""   # use published octomap_frame
        ))
    return resp


def call_compute_vis_value_srv_xytheta(x, y, theta, mode):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = PROSILICA_TRANS_Z
    q = quaternion_from_euler(0, 0, theta)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return call_compute_vis_value_srv(pose, mode)


def get_poses_from_topomap(topomap_fiilename):
    get_topomap_resp = call_srv("get_topomap", GetTopologicalMap, (topomap_fiilename, ""))
    if not get_topomap_resp:
        return None

    poses = []
    for place in get_topomap_resp.topomap.places:
        for view in place.views:
            pose = Pose()
            pose.position.x = view.pose.x
            pose.position.y = view.pose.y
            pose.position.z = PROSILICA_TRANS_Z
            q = quaternion_from_euler(0, 0, view.pose.theta)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            poses.append(pose)

    return poses


def apply_pmap_bbx_filter(poses, pmap_filename, margin):
    '''
        margin: in meters
    '''
    get_pmap_resp = call_srv("get_pmap", GetProbabilityMap, (pmap_filename, ""))
    if not get_pmap_resp:
        sys.exit(1)
    # compute pmap bbx
    minX = get_pmap_resp.pmap.info.origin.position.x - margin
    maxX = get_pmap_resp.pmap.info.origin.position.x + \
        (get_pmap_resp.pmap.info.width * get_pmap_resp.pmap.info.resolution) + margin
    minY = get_pmap_resp.pmap.info.origin.position.y - margin
    maxY = get_pmap_resp.pmap.info.origin.position.y + \
        (get_pmap_resp.pmap.info.height * get_pmap_resp.pmap.info.resolution) + margin

    new_poses = []
    for pose in poses:
        if pose.position.x > minX and pose.position.x < maxX and pose.position.y > minY and pose.position.y < maxY:
            new_poses.append(pose)

    return new_poses


def call_compute_vis_values_srv(poses, mode):
    resp = call_srv("compute_vis_values", ComputeVisibilityValues, (
        poses,
        mode,
        "",  # use published pmap_filaname
        "",  # use published pmap_frame
        "",  # use published octomap_filaname
        ""   # use published octomap_frame
        ))
    return resp
