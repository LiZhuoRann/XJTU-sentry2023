// #include "pcl_conversions/pcl_conversions.h"
// #include "ros/init.h"
// #include "ros/node_handle.h"
// #include <ros/ros.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <sensor_msgs/PointCloud2.h> 

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "pcl_publisher");
//     ros::NodeHandle nh;

//     ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lidar_cloud",1);
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     sensor_msgs::PointCloud2 output;
//     output.header.frame_id = "map";
//     pcl::io::loadPCDFile("/home/xjturm/shaobing/src/sentry_slam/FAST_LIO/PCD/XDU.pcd",cloud);
//     pcl::toROSMsg(cloud,output);
//     ros::Rate loop_rate(10);

//     while(ros::ok()) {
//         cloud_pub.publish(output);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
 
int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_load");  //初始化节点，创建节点名称
  ros::NodeHandle nh;	//节点处理句柄
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1); //发布名称为pcl_out的话题，消息队列长度为1，消息类型为sensor_msgs::PointCloud2
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile ("/home/xjturm/shaobing/src/sentry_slam/FAST_LIO/PCD/XDU.pcd", cloud); //修改自己pcd文件所在路径
  //Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output); //转换为ROS的消息类型
  output.header.frame_id = "map";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
//！！！这一步需要注意，是后面rviz的 fixed_frame  !!!敲黑板，画重点。
  ros::Rate loop_rate(10);  //控制发布的信息的快慢,即循环内sleep的间隔

  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();  //监听反馈函数
    loop_rate.sleep();
  }
  return 0;
}

