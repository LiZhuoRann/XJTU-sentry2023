#include <cstring>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include "protocol.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/time.h"
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

//  action client 我们要连接到 move_base server

navCommand_t navGoal;

// 数据串行化
template<class type> char *serialize(type &data) {
    size_t size = sizeof(data);
    char *data_char = new char[size];
    memcpy(data_char, &data, size);
    return data_char;
}

// 数据反串行化
template<class type> type deserialize(const char *data_str) {
    type readData;
    memcpy(&readData, data_str, sizeof(readData));
    return readData;
}

geometry_msgs::PoseWithCovarianceStamped fillInitialPose(const char* color_) {
    geometry_msgs::PoseWithCovarianceStamped initialPose;
    if(std::strcmp(color_, "RED") == 0) {   // color_ equals to "RED"
        initialPose.pose.pose.position.x = 0;
        initialPose.pose.pose.position.y = 0;
        initialPose.pose.pose.position.z = 0;
        tf::Quaternion quat;
        quat.setRPY(0,0,0);
        initialPose.pose.pose.orientation.w = quat.w();
        initialPose.pose.pose.orientation.x = quat.x();
        initialPose.pose.pose.orientation.y = quat.y();
        initialPose.pose.pose.orientation.z = quat.z();
        initialPose.header.stamp = ros::Time::now();
        initialPose.header.frame_id = 'map';
    }
    if(std::strcmp(color_, "BLUE") == 0) {
        initialPose.pose.pose.position.x = 3.6;
        initialPose.pose.pose.position.y = -4.0;
        initialPose.pose.pose.position.z = 0;
        tf::Quaternion quat;
        quat.setRPY(0,0,180);
        initialPose.pose.pose.orientation.w = quat.w();
        initialPose.pose.pose.orientation.x = quat.x();
        initialPose.pose.pose.orientation.y = quat.y();
        initialPose.pose.pose.orientation.z = quat.z();
        initialPose.header.stamp = ros::Time::now();
        initialPose.header.frame_id = 'map';
    }
    return initialPose;
}

navCommand_t fake_navGoal = {
    .frame_header = 0x20,
    .coo_x = 0.2,
    .coo_y = 0.2,
    .stop = 0,
    .color = 1,
    .reverse_1 = 0,
    .reverse_2 = 0,
    .reverse_3 = 0,
    .reverse_4 = 0,
    .reverse_5 = 0,
    .frame_tail = 0x21,
};

class TransmitNode_t {
private:
    ros::NodeHandle nh;
    ros::Publisher pub_initialPose;
    ros::Subscriber sub_serial;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    bool debug_mode = false;
public:
    // 构造函数中使用初始化列表，来初始化成员类 ac
    TransmitNode_t() : ac("move_base", true) {
        nh.param("debug", debug_mode, false);
        if(debug_mode) {
            nh.param("coo_x", fake_navGoal.coo_x);
            nh.param("coo_y", fake_navGoal.coo_y);
            nh.param("stop", fake_navGoal.stop);
            nh.param("color", fake_navGoal.color);
        }

        // 创建 Subscriber 对象，订阅 serial_data 主题
        sub_serial = nh.subscribe<std_msgs::String>("/serial_data", 100, &TransmitNode_t::serialDataCallback, this);
        // 根据机器人颜色不同，发布 /initialPose 话题
        pub_initialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialPose",1,true);
        // 等待 server 变为可用
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        if(!ac.isServerConnected()) {
            ROS_ERROR("move_base server not connected!!!!");
        }
    }

    // 收到串口数据后的回调函数
    void serialDataCallback(const std_msgs::String::ConstPtr &msg) {
        // 处理串口数据，这里只是假设打印接收到的数据
        navGoal = deserialize<navCommand_t>(msg->data.c_str());
        // 检查帧头帧尾
        if ((navGoal.frame_header != 0x20) || (navGoal.frame_tail != 0x21)){
            return;
        }

        // 决策指令
        static float last_x_cmd = 0;
        static float last_y_cmd = 0;
        // ros::param::set("/real_world/x", navGoal.coo_x);
        // ros::param::set("/real_world/y", navGoal.coo_y);
        // 检测目标指令有没有发生变化
        navGoal = fake_navGoal;
        if(navGoal.coo_x != last_x_cmd || navGoal.coo_y != last_y_cmd) {
            move_base_msgs::MoveBaseGoal goal;
            // 设置目标点的位置和方向
            goal.target_pose.pose.position.x = navGoal.coo_x;
            goal.target_pose.pose.position.y = navGoal.coo_y;
            // 设置目标点的 frame
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal);
        }
        if(navGoal.stop) {
            ROS_INFO("stop navigating! shoot enemies!");
            ac.cancelGoal();
            ros::param::set("/real_world/is_aiming", navGoal.stop);
            // 如果正在自瞄，就取消当前正在追踪的目标
            // int navTerminate = 0;
            // ros::param::get("/real_world/is_aiming", navTerminate);
            // if(navTerminate) {
                // ac.cancelGoal();
            // }
        }

        // 初始化出生点，发布 /initialpose
        static bool initialized = false;
        if (!initialized && navGoal.color == 1) {    // 1=RED
            initialized = true;
            pub_initialPose.publish(fillInitialPose("RED"));
        } 
        else if (!initialized && navGoal.color == 2) {    // 2=BLUE
            initialized = true;
            pub_initialPose.publish(fillInitialPose("BLUE"));
        } 
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener_mcu");
    TransmitNode_t TransmitNode;
    ros::spin(); // 进入消息循环
    return 0;
}