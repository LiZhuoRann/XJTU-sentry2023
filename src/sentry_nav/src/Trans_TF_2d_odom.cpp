/**
 * @file Trans_TF_2d_odom.cpp
 * @author your name (you@domain.com)
 * @brief 广播 tf 变换
 * @version 0.1
 * @date 2024-03-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <ros/ros.h>  
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// tf chage https://zhuanlan.zhihu.com/p/340016739


int main(int argc, char** argv){
  ros::init(argc, argv, "Trans_TF_2d_odom");

  ros::NodeHandle node;

  tf::TransformListener listener;       // tf变换监听器
  tf::TransformBroadcaster broadcaster; // tf变换广播器
  tf::Transform transform_broadcaster;  // 被广播的tf，存放转换信息

  ros::Rate rate(1000);  // 1kHz

  while (node.ok()){
    tf::StampedTransform transform_listener;
    
    try{
      /**
       * @brief 查询从 camera_init 到 map 坐标系的变换关系
       * @param target_frame The frame to which data should be transformed 
       * @param source_frame The frame where the data originated 
       * @param time ros::Time(0)表示获取最近的可用变换
       * @param transform 查询结果存储在 transform_listener 对象中
       * @note 注意参数顺序，第一个参数是 target_frame, 第二个参数是 source_frame
      */
      listener.lookupTransform("map", "camera_init",  
                               ros::Time(0), transform_listener);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    float robot_pose_x=transform_listener.getOrigin().x();
    float robot_pose_y=transform_listener.getOrigin().y();
    // float robot_pose_z=0;
    // float robot_oriation_x=transform_listener.getRotation().getX();
    // float robot_oriation_y=transform_listener.getRotation().getY();
    float robot_oriation_z=transform_listener.getRotation().getZ();
    float robot_oriation_w=transform_listener.getRotation().getW();

    /**
     * @brief  设置变换的平移部分
     * @param  v 类型为 tf::Vector3& 的平移向量
     * @note   感觉想法挺好的，把雷达坐标系拍到了地面（z=0），但是最后没有用这个
     */ 
    transform_broadcaster.setOrigin( tf::Vector3(robot_pose_x, robot_pose_y, 0.0) );
    /**
     * @brief  设置变换的旋转部分
     * @param  q 类型为 tf::Quaternion& 的旋转四元数
     * @note  tf::Quaternion 构造函数参数的顺序为 x,y,z,w, 不同于 Eigen::Quaterniond 的 w,x,y,z
     */
    transform_broadcaster.setRotation( tf::Quaternion(0, 0, robot_oriation_z, robot_oriation_w) );
    /**
     * @brief 广播 tf 变换
     * @param frame_id_      ：map            < The frame_id of the coordinate frame  in which this transform is defined
     * @param child_frame_id_：camera_init_2d < The frame_id of the coordinate frame this transform defines
     */
    broadcaster.sendTransform(tf::StampedTransform(transform_broadcaster, ros::Time::now(), "map", "camera_init_2d"));

    rate.sleep();
  }
  return 0;
};