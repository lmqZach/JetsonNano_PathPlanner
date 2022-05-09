#!/usr/bin/env python

import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)


def tag_callback(msg):
    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    tag_map = {0: 1, 42: 2}  # initialize tag maps here, tag_id: entry
    f = []  # initialize measurements here
    for tag_id, tag_pose in zip(msg.ids, msg.detections):
        tm = tag_pose.matrix
        tm = np.array(tm).reshape(4, 4)
        origin = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        # p(origin in camera frame) = transformation matrix * p(origin)
        relative_pos = np.matmul(tm, origin).reshape(4)
        relative_x, relative_y = relative_pos[0], relative_pos[2]
        entry = tag_map[tag_id]
        # map from camera frame to robot frame
        f += [entry, relative_y, -relative_x, np.arctan2(tm[2, 2], tm[0, 2])]
        pose_msg.pose.matrix = f
    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
