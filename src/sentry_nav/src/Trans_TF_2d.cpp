/**
 * @file Trans_TF_2d.cpp
 * @author your name (you@domain.com)
 * @brief  由于 fast_lio_localization 输出的 body frame 是当前机器人在三维点云坐标系下的位姿，
    而 move_base 需要的 map frame 是二维栅格地图坐标系下的坐标 body_2d，因此需要进行坐标系转换。
    把 body frame 中的 (x,y,z) 取 (x,y,0) 赋给 body_2d 即可
    把 body frame 中的四元数 (x,y,z,w) 取 (0,0,z,w) 赋给 body_2d
 * @version 0.1
 * @date 2024-03-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "Trans_TF_2d");

  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform_broadcaster;
  ros::Duration(1.0).sleep();

  ros::Rate rate(1000);
  while (node.ok()){
    tf::StampedTransform transform_listener;
    
    try{
      listener.lookupTransform("map", "body",  
                               ros::Time(0), transform_listener);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    // 只
    float robot_pose_x=transform_listener.getOrigin().x();
    float robot_pose_y=transform_listener.getOrigin().y();
    float robot_oriation_z=transform_listener.getRotation().getZ();
    float robot_oriation_w=transform_listener.getRotation().getW();

    transform_broadcaster.setOrigin( tf::Vector3(robot_pose_x, robot_pose_y, 0.0) );
    transform_broadcaster.setRotation( tf::Quaternion(0, 0, robot_oriation_z, robot_oriation_w) );
    broadcaster.sendTransform(tf::StampedTransform(transform_broadcaster, ros::Time::now(), "map", "body_2d"));

    rate.sleep();
  }
  return 0;
};
