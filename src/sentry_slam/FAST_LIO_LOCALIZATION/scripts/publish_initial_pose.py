#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import argparse

import rospy
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

class arg_:
    def __init__(self, x,y,z,pitch,yaw,roll):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

if __name__ == '__main__':
    # 使用argparse库来解析命令行参数
    # parser = argparse.ArgumentParser()
    # parser.add_argument('x', type=float)
    # parser.add_argument('y', type=float)
    # parser.add_argument('z', type=float)
    # parser.add_argument('yaw', type=float)
    # parser.add_argument('pitch', type=float)
    # parser.add_argument('roll', type=float)
    # args = parser.parse_args()
    x = rospy.get_param('/publish_initial_pose/x',0)
    y = rospy.get_param('/publish_initial_pose/y',0)
    z = rospy.get_param('/publish_initial_pose/z',0)
    pitch = rospy.get_param('/publish_initial_pose/pitch',0)
    yaw = rospy.get_param('/publish_initial_pose/yaw',0)
    roll = rospy.get_param('/publish_initial_pose/roll',0)
    args = arg_(x,y,z,pitch,yaw,roll)

    # 发布 PoseWithCovarianceStamped 类型的话题 /initialpose
    rospy.init_node('publish_initial_pose')
    pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    # 转换为pose
    quat = tf.transformations.quaternion_from_euler(args.roll, args.pitch, args.yaw)
    xyz = [args.x, args.y, args.z]

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    initial_pose.header.stamp = rospy.Time().now()
    initial_pose.header.frame_id = 'map'

    rospy.sleep(1)
    rospy.loginfo('Initial Pose: {} {} {} {} {} {}'.format(
        args.x, args.y, args.z, args.yaw, args.pitch, args.roll, ))
    # 成功发布初始位姿
    pub_pose.publish(initial_pose)
