#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include "protocol.h"
#include "ros/duration.h"
#include "ros/time.h"
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
// #include <iostream>

#define LOG_ENABLE 0

navInfo_t navInfo;
ros::Publisher pub_navInfo;
bool gameMode;
ros::Time goalTimeStamp;

void goal_calllback(const move_base_msgs::MoveBaseGoal& msg) {
    auto x = msg.target_pose.pose.position.x;
    auto y = msg.target_pose.pose.position.y;
    ROS_INFO("\n Target Coordinates: x:[%f], y:[%f]", x, y);
    goalTimeStamp = msg.target_pose.header.stamp;
}

void status_callback(const move_base_msgs::MoveBaseActionResult& msg)
{	
	if(msg.status.status == 3)
	{
        navInfo.is_reach = 1;
        navInfo.x_speed = navInfo.y_speed = 0;
		std::cout << "the goal was achieved successfully!" << std::endl;
	}
}

// 回调函数，处理接收到的/cmd_vel消息
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // 24.3.20 速度指令长时间不变，认为导航到位或者出现问题了，速度指令给0
    static float linear_x = 0;
    static float linear_y = 0;
    static int count = 0;
    if (fabsf(msg->linear.x - linear_x) > 1e-5 || fabsf(msg->linear.y - linear_y) > 1e-5) {
        // ROS_INFO("I'm here!!!!");
        count = 0;
        navInfo.is_reach = 0x00;
    } else {
        count ++;
    }
    if(count > 10U) {
        navInfo.is_reach = 0x01;
        return ;
    }
    // 提取线速度
    linear_x = msg->linear.x;
    linear_y = msg->linear.y;
    // 打印输出
#if LOG_ENABLE
    ROS_INFO("\nReceived cmd_vel: Linear_x = %f, Linear_y = %f\n", linear_x, linear_y);
#endif
    navInfo.x_speed = linear_x;
    // 由于雷达是倒装的，坐标系变反了，这里先加个负号，很粗暴，之后再说
    // TODO: 静态 TF 变换，速度指令 from lidar frame to chassis frame.
    navInfo.y_speed = -linear_y;
}

// map frame: 建图起点 右下角(0,0)，左下角（-0.3, -4），
//            另一边出生点 左上角(3.6,)，右上角(3.7,, -0.2)
// TODO: 点到墙里就寄了
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "listener_cmdvel");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    // 比赛模式 
    // gameMode=true，接收由电控发来的坐标指令
    // gameMode=false, 使用 rviz 标点调试导航
    nh.param("gameMode", gameMode, false);

    // 订阅 /cmd_vel主题，指定回调函数
    ros::Subscriber sub_cmd_vel = nh.subscribe("/smooth_cmd_vel", 100, cmdVelCallback);
    ros::Subscriber sub_movebase_result = nh.subscribe("move_base/result", 10, status_callback);
    ros::Subscriber sub_movebase_goal = nh.subscribe("move_base/goal", 10, goal_calllback);
    // 发布 std_msgs::string 类型的导航状态信息
    pub_navInfo = nh.advertise<std_msgs::String>("cmd_vel_string", 100);
    // 监听 map2odom 的 TF 变换
    tf::TransformListener listener;

    auto loopRate = ros::Rate(1000);
    while(ros::ok()) {
        // 循环监听ROS消息
        ros::spinOnce();
        
        // 填充帧头帧尾
        navInfo.frame_header = 0x20;
        navInfo.frame_tail = 0x21;

        // 到达目标位置
        if(navInfo.is_reach) {
            navInfo.x_speed = navInfo.y_speed = 0;
        }
        // 规定时间内没有导航成功，就别动了
        if(ros::Time::now() - goalTimeStamp > ros::Duration(10)) {
            navInfo.is_reach = 0;
            navInfo.x_speed = navInfo.y_speed = 0;
        }
#if 0
        // 查询导航状态 
        // TODO: 这里状态始终是 LOST
        // actionlib::SimpleClientGoalState state = ac.getState();
        // switch (state.state_) {
        //     case actionlib::SimpleClientGoalState::ACTIVE: {
        //         navInfo.is_reach = 0;
        //         ; // 这里应该在回调函数里给速度指令赋值
        //         break;
        //     } 
        //     case actionlib::SimpleClientGoalState::SUCCEEDED: {
        //         // 成功到达目标点
        //         navInfo.x_speed = navInfo.y_speed = 0;
        //         navInfo.is_reach = 1;
        //         break;
        //     }
        //     case actionlib::SimpleClientGoalState::ABORTED: {
        //         // 导航终止
        //         navInfo.x_speed = navInfo.y_speed = 0;
        //     }
        // }
#endif
   
        // 查询当前坐标
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/body",
                                ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        navInfo.coo_x_current = transform.getOrigin().x();
        navInfo.coo_y_current = transform.getOrigin().y();
        // std::cout << "current speed cmd: vx= [" << navInfo.x_speed << "], vy= [" << navInfo.y_speed << "]" << std::endl;
        // ROS_INFO("\n is reach: %x", navInfo.is_reach);
        // std::cout << "is reach: [" << static_cast<char>(navInfo.is_reach) << "]" << std::endl;
#if LOG_ENABLE
        std::cout << "Action Status: " << state.toString().c_str() << std::endl; 
        std::cout << "current coordinates in map: x= [" << navInfo.coo_x_current << "], y= [" << navInfo.coo_y_current << " ]"<< std::endl;
        std::cout << std::endl;
#endif

        // 任务频率 1kHz
        char data[32];
        std::memcpy(data,&navInfo,sizeof(navInfo_t));
        std_msgs::String navInfoString;
        navInfoString.data = std::string(data,sizeof(navInfo_t));
        pub_navInfo.publish(navInfoString);
        // printf("\n[navInfo] navInfo: ");
        // for (int i = 0; i < navInfoString.data.length(); i++)
        // {
        //     printf("%x", navInfoString.data[i]);
        // }
        // ROS_INFO("\n navInfo: %s", navInfoString.data.c_str());

        loopRate.sleep();
    }
    return 0;
}
