#!/usr/bin/python3
# coding=utf8

# 这个线程的核心就是通过变换矩阵符合运算，得到 map->odom->baselink 的 TF 
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
'''
nav_msgs::Odometry 是ROS（机器人操作系统）中用于表示机器人位姿和运动信息的消息类型。它包含以下主要信息：
 - header：消息头，包含时间戳和坐标系信息。位姿（pose）应该在header.frame_id指定的坐标系中给出。
 - child_frame_id：子坐标系的ID，通常是机器人的基底（base）坐标系。速度（twist）应该在child_frame_id指定的坐标系中给出。
 - pose：机器人在odometric坐标系中的估计位置，以及该位置估计的协方差。geometry_msgs/PoseWithCovariance类型。
 - twist：机器人在child_frame_id坐标系中的速度，以及该速度估计的协方差。geometry_msgs/TwistWithCovariance类型。
'''
baselink = None
cur_map_to_odom = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom

    br = tf.TransformBroadcaster()
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)

        # TODO 这里注意线程安全
        cur_odom = copy.copy(cur_odom_to_baselink)
        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         'camera_init', 'map')
        if cur_odom is not None:
            # 发布全局定位的 odometry
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom)
            # 这里 T_map_to_odom 短时间内变化缓慢 暂时不考虑与 T_odom_to_base_link 时间同步
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            # pose：机器人在odometric坐标系中的估计位置，以及该位置估计的协方差
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            # twist：机器人在child_frame_id坐标系中的速度，以及该速度估计的协方差
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = 'map'
            localization.child_frame_id = 'body'
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)


def cb_save_cur_odom(odom_msg):
    # 把订阅到的 /Odomtery 保存到全局变量 cur_odom_to_baselink 中
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    # 把订阅到的 /map_to_odom 保存到全局变量 cur_map_to_odom 中
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    # 订阅 /Odometry 和 /map_to_odom 话题，注册回调函数
    # /Odometry 表示 camera_init 到 body 的 TF，由 FAST_LIO 发布 
    
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()
