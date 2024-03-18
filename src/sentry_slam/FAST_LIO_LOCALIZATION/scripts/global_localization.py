#!/usr/bin/python3
# coding=utf8

from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None

# 位姿信息 -> 变换矩阵
def pose_to_mat(pose_msg):
    # 矩阵相乘，表示了一个旋转和平移的组合变换
    return np.matmul(
        # 将 3D 位置向量转换为一个平移变换矩阵
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        # 将四元数转换为一个旋转变换矩阵
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )

# 点云 -> array
def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    # 新建一个数组，用于存储点云中每个点的 x、y，z 坐标
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc

'''
/**
 * @brief 使用迭代最近点（Iterative Closest Point，ICP）算法来对两个点云进行配准
 * @param pc_scan 待配准点云
 * @param pc_map  参考点云
 * @param initial 初始变换矩阵
 * @param scale   缩放因子
 */
 '''
def registration_at_scale(pc_scan, pc_map, initial, scale):
    # 调用 oepn3d 库函数，进行 ICP 配准
    result_icp = o3d.pipelines.registration.registration_icp(
        # 源点云和目标点云都被降采样，以提高配准速度
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        # 最大配准距离：1.0*scale
        1.0 * scale, initial,
        # 变换估计方法：点对点（Point-to-Point）
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        # 收敛条件：最大迭代次数为20。
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
    )
    # 返回配准结果的变换矩阵和适应度值。
    # 变换矩阵描述了如何将 pc_scan 变换到 pc_map 的坐标系中，
    # 适应度值描述了配准的质量，值越大表示配准越好。
    return result_icp.transformation, result_icp.fitness

# 计算一个SE(3)变换矩阵的逆
def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R 计算旋转部分的逆。在SE(3)变换矩阵中，左上角的3x3子矩阵表示旋转。
    # 旋转矩阵的逆等于它的转置，所以这里直接取trans的左上角3x3子矩阵的转置。
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t 计算平移部分的逆。在SE(3)变换矩阵中，最右列的前三个元素表示平移。
    # 平移向量的逆等于它的负值，但是这个负值需要在旋转的坐标系中表示，
    # 所以这里先将trans的左上角3x3子矩阵（即旋转矩阵）的转置与trans的最右列的前三个元素（即平移向量）相乘，然后取负值。
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse

# 发布点云话题
def publish_point_cloud(publisher, header, pc):
    # 创建一个 numpy array，定义长度和数据类型
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    # 填充 data
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    # 检查pc是否包含强度信息。如果pc的列数等于4，那么它就包含强度信息
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    # ros_numpy.msgify函数将data转换为一个PointCloud2消息，然后发布消息
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)

'''
/**
 * @brief 提取全局地图中在当前机器人视野（Field of View，FOV）内的点云
 * @param global_map       全局地图的点云
 * @param pose_estimation  机器人的估计位姿 map -> odom
 * @param cur_odom         当前的里程计读数
 * @note  在这里选择合适的FOV，过滤扫描到的点云中的车身
 */
 '''
def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 当前scan原点的位姿
    # base -> map = (map -> odom -> base)^{-1}
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    # 在数组的每一行后面添加一个1，得到一个齐次坐标表示的点云
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    # 得到在机器人坐标系中的点云k
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )

    # 发布fov内点云
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    # 创建当前激光扫描点云的一个副本 TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    # 提取全局地图中在当前机器人视野内的点云
    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)
    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time: {}'.format(toc - tic))
    rospy.loginfo('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])


def thread_localization():
    global T_map_to_odom
    while True:
        # 每隔一段时间进行全局定位
        rospy.sleep(1 / FREQ_LOCALIZATION)
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        global_localization(T_map_to_odom)


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.95

    # FOV(rad), modify this according to your LiDAR type
    FOV = 6.28

    # The farthest distance(meters) within FOV
    FOV_FAR = 30

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    # /cloud_registered 和 /Odometry 都由 laserMapping 线程发布
    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 初始化
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')

        # 等待初始位姿
        # 不给定 timeout 参数时，默认会一直等待
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    _thread.start_new_thread(thread_localization, ())

    rospy.spin()
