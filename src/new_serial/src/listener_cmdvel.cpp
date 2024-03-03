#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "../include/listner_cmdvel.h"
#include <std_msgs/String.h>
#include <iostream>

ros::Publisher cmd_pub;

// 回调函数，处理接收到的/cmd_vel消息
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // 提取线速度和角速度
    float linear_x = msg->linear.x;
    float linear_y = msg->linear.y;
    float angular_z = msg->angular.z;
    // 打印输出
    ROS_INFO("Received cmd_vel: Linear_x = %f, Linear_y = %f, Angular = %f", linear_x, linear_y, angular_z);
    char data[14];
    msg_cmd_t msg_cmd;
    msg_cmd.markHead = 0x5e;
    msg_cmd.markTail = 0x71;
    msg_cmd.speedX   = -linear_y;
    msg_cmd.speedY   = linear_x;
    msg_cmd.speedZ   = angular_z;
    std::memcpy(data,&msg_cmd,sizeof(msg_cmd_t));
    std_msgs::String msgs;
    msgs.data = std::string(data,sizeof(msg_cmd_t));
    cmd_pub.publish(msgs);
    // std::cout<<std::string(data,sizeof(msg_cmd_t));
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "listener_cmdvel");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 订阅/cmd_vel主题，指定回调函数
    // ros::Subscriber sub = nh.subscribe("/cmd_vel", 100, cmdVelCallback);

    ros::Subscriber sub = nh.subscribe("/smooth_cmd_vel", 100, cmdVelCallback);

    cmd_pub = nh.advertise<std_msgs::String>("cmd_vel_string", 100);

    // 循环监听ROS消息
    ros::spin();

    return 0;
}
