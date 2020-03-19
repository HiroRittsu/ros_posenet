#!/usr/bin/env python
# coding: UTF-8
import rospy
from ros_posenet.msg import Poses
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')


def callback(poses):
    plt.cla()
    for pose in poses.poses:
        for k in pose.keypoints:
            print k.position.z
            # ax.scatter(k.position.x, k.position.y, k.position.z)
            plt.scatter(k.position.x, -k.position.y)
    plt.pause(.001)

    # plt.pause(.01)


rospy.init_node('pose_viewer')
rospy.Subscriber('/ros_posenet/poses', Poses, callback)

rospy.spin()
