#include <ros/ros.h>
#include <std_msgs/String.h>
#include "protocol.h"
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

//  action client 我们要连接到 move_base server
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

// 数据串行化
template<class type>
char *serialize(type &data) {
    size_t size = sizeof(data);
    char *data_char = new char[size];
    memcpy(data_char, &data, size);
    return data_char;
}

// 数据串行化
template<class type>
type deserialize(const char *data_str) {
    type readData;
    memcpy(&readData, data_str, sizeof(readData));
    return readData;
}

// 数据反串行化
void serialDataCallback(const std_msgs::String::ConstPtr &msg) {
    // 处理串口数据，这里只是假设打印接收到的数据
    auto data = msg->data.c_str();

    auto navGoal = deserialize<navCommand_t>(data);
    if ((navGoal.frame_header != 0x20) || (navGoal.frame_tail != 0x21)){
        return;
    }
    printf("帧头帧尾匹配成功\n");
    ros::param::set("/real_world/x", navGoal.coo_x);
    ros::param::set("/real_world/y", navGoal.coo_y);
    if(navGoal.stop) {
        ros::param::set("/real_world/is_aiming", navGoal.stop);
    }

    move_base_msgs::MoveBaseGoal goal;
    // 设置目标点的 frame
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    // 设置目标点的位置和方向
    goal.target_pose.pose.position.x = navGoal.coo_x;
    goal.target_pose.pose.position.x = navGoal.coo_y;

    // ac.sendGoal(goal);
    // ac.waitForResult();

    // 如果正在自瞄，就取消当前正在追踪的目标
    // int navTerminate = 0;
    // ros::param::get("/real_world/is_aiming", navTerminate);
    // if(navTerminate) {
    //     ac.cancelGoal();
    // }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener_mcu");
    ros::NodeHandle nh;

    // 创建 Subscriber 对象，订阅 serial_data 主题
    ros::Subscriber serial_sub = nh.subscribe<std_msgs::String>("/serial_data", 100, serialDataCallback);

    // 等待 server 变为可用
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    if(!ac.isServerConnected()) {
        ROS_ERROR("move_base server not connected!!!!");
    }
    mux
    ros::spin(); // 进入消息循环

    return 0;
}