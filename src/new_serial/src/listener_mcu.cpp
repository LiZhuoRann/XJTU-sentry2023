#include <ros/ros.h>
#include <std_msgs/String.h>
#include "../include/listner_mcu.h"
#include <sstream>
#include <iostream>

template<class type>
char *serialize(type &data) {
    size_t size = sizeof(data);
    char *data_char = new char[size];
    memcpy(data_char, &data, size);
    return data_char;
}

template<class type>
type deserialize(const char *data_str) {
    type readData;
    memcpy(&readData, data_str, sizeof(readData));
    return readData;
}

void serialDataCallback(const std_msgs::String::ConstPtr &msg) {
    // 处理串口数据，这里只是假设打印接收到的数据
    auto data = msg->data.c_str();

    auto sentryInfo = deserialize<msg_mcu_t>(data);
    if ((sentryInfo.head != 0x6f) || (sentryInfo.tail != 0x83)){
        return;
    }
    printf("帧头帧尾匹配成功\n");
    ros::param::set("/RobotStatus/hp", sentryInfo.hp);
    ros::param::set("/RobotStatus/bullet", sentryInfo.bullet);
    ros::param::set("/RobotStatus/post",sentryInfo.post);
    ros::param::set("/RobotStatus/time", sentryInfo.time);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener_mcu");
    ros::NodeHandle nh;

    // 创建 Subscriber 对象，订阅 serial_data 主题
    ros::Subscriber serial_sub = nh.subscribe<std_msgs::String>("/serial_data", 100, serialDataCallback);

    ros::spin(); // 进入消息循环

    return 0;
}