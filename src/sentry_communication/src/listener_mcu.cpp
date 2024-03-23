#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PointStamped.h"
#include "protocol.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/console.h>

navCommand_t navGoal;
const float RED_SENTRY_INIT_POS_X = 0;
const float RED_SENTRY_INIT_POS_Y = 0;
const float RED_SENTRY_INIT_POS_Z = 0;
const float RED_SENTRY_INIT_ORIEN_ROLL = 0;
const float RED_SENTRY_INIT_ORIEN_YAW = 0;
const float RED_SENTRY_INIT_ORIEN_PITCH = 0;
const float BLUE_SENTRY_INIT_POS_X = 0;
const float BLUE_SENTRY_INIT_POS_Y = 0;
const float BLUE_SENTRY_INIT_POS_Z = 0;
const float BLUE_SENTRY_INIT_ORIEN_ROLL = 0;
const float BLUE_SENTRY_INIT_ORIEN_YAW = 0;
const float BLUE_SENTRY_INIT_ORIEN_PITCH = 0;

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

geometry_msgs::PoseWithCovarianceStamped fillInitialPose(const char* color_);

navCommand_t fake_navGoal = {
    .frame_header = 0x20,
    .frame_tail   = 0x21,
};

double param_x;
double param_y;
int param_stop;
int param_color;

class TransmitNode_t {
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    ros::Publisher pub_initialPose;
    ros::Publisher pub_navState;
    ros::Timer timer;
    ros::Subscriber sub_serial;
    ros::Subscriber sub_fakeGoal;
    ros::Time statrTimeStamp;
    bool debug_mode = false;
public:
    // 构造函数中使用初始化列表，来初始化成员类 ac
    TransmitNode_t() : ac("move_base", true) {
#if 0  
        //设置日志等级
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal) ) {
            ros::console::notifyLoggerLevelsChanged();
        }
#endif
        nh.param("/listener_mcu/debug", debug_mode, false);
        if(debug_mode) {
            // 由于结构体成员类型是 C 风格，定义几个 param_xxx 变量接一下
            nh.param("/listener_mcu/coo_x", param_x, 0.0);
            nh.param("/listener_mcu/coo_y", param_y, 0.0);
            nh.param("/listener_mcu/stop", param_stop, 0);
            nh.param("/listener_mcu/color", param_color,0);
        }
        fake_navGoal.coo_x = param_x;
        fake_navGoal.coo_y = param_y;
        fake_navGoal.stop = param_stop;
        fake_navGoal.color = param_color;
        ROS_INFO("debug_mode (as integer): [%d], param_x:[%f], fake_navGoal.coo_x:[%f]",debug_mode, param_x, fake_navGoal.coo_x);
        // 等待 server 变为可用
        while(!ac.waitForServer(ros::Duration(1.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        if(!ac.isServerConnected()) {
            ROS_ERROR("move_base server not connected!!!!");
        } else {
            ROS_INFO("move_base server connected!!!!");
        }
        // 创建 Subscriber 对象，订阅 serial_data 主题
        sub_serial = nh.subscribe<std_msgs::String>("/serial_data", 100, &TransmitNode_t::serialDataCallback, this);
        // sub_fakeGoal = nh.subscribe("/fakeNavGoal", 1, &TransmitNode_t::fakeNavGoalCallback);
        // 根据机器人颜色不同，发布 /initialPose 话题
        pub_initialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1,true);
        pub_navState = nh.advertise<std_msgs::String>("/move_base1/navState", 1);
        // 定时发布
        timer = nh.createTimer(ros::Duration(0.1), boost::bind(&TransmitNode_t::timerUpdateCallback, this));
        statrTimeStamp = ros::Time::now();
    }

    // 收到串口数据后的回调函数
    void serialDataCallback(const std_msgs::String::ConstPtr &msg) {
        if(ros::Time::now() - statrTimeStamp < ros::Duration(2)) {
            return ;
        }
        // 处理串口数据，这里只是假设打印接收到的数据
        navGoal = deserialize<navCommand_t>(msg->data.c_str());
        if(debug_mode) {
            // debug 模式，navGoal 由参数文件给定
            navGoal = fake_navGoal;
        }

        // 检查帧头帧尾
        if ((navGoal.frame_header != 0x20) || (navGoal.frame_tail != 0x21)){
            return;
        }

        // 初始化出生点，发布 /initialpose
        static bool initialized = false;
        if (!initialized && navGoal.color == 1) {    // 1=RED
            ROS_INFO("*************** Robot Color is RED! ***************");
            initialized = true;
            pub_initialPose.publish(fillInitialPose("RED"));
        } 
        else if (!initialized && navGoal.color == 2) {    // 2=BLUE
            ROS_INFO("*************** Robot Color is BLUE! ***************");
            initialized = true;
            pub_initialPose.publish(fillInitialPose("BLUE"));
        } 
        
        // 决策指令
        static float last_x_cmd = 1;
        static float last_y_cmd = 1;
        // ros::param::set("/real_world/x", navGoal.coo_x);
        // ros::param::set("/real_world/y", navGoal.coo_y);
        // 检测目标指令有没有发生变化
        static bool init = true;
        if(navGoal.coo_x != last_x_cmd || navGoal.coo_y  != last_y_cmd || init) {
            if(init) {
                ros::Duration(2).sleep();
            }
            init = false;
            move_base_msgs::MoveBaseGoal goal;
            // 设置目标点的位置和方向
            goal.target_pose.pose.orientation.w = 1;
            // 设置目标点的 frame
            last_x_cmd = navGoal.coo_x; 
            last_y_cmd = navGoal.coo_y;
            tf::StampedTransform tf;
            tf::TransformListener tf_listener;
            geometry_msgs::PointStamped pointInMapFrame;
            geometry_msgs::PointStamped pointInWorldFrame;
            pointInWorldFrame.point.x = navGoal.coo_x;
            pointInWorldFrame.point.y = navGoal.coo_y;
            pointInWorldFrame.point.z = 0;
            pointInWorldFrame.header.frame_id = "realWorld";
            pointInWorldFrame.header.stamp = ros::Time::now();
            bool tferr = true;
            while(tferr) {
                tferr = false;
                try{
                    // tf 目标点平移
                    tf_listener.transformPoint("map", pointInWorldFrame, pointInMapFrame);
                }
                catch (tf::TransformException &ex) {
                    tferr = true;
                    static uint32_t count = 0;
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.1).sleep();
                    ROS_INFO("Trying to transform point: count %d", count++);
                }
            }

            ROS_INFO("Point in map frame : x:[%f], y:[%f], z:[%f]", \
                pointInMapFrame.point.x, pointInMapFrame.point.y, pointInMapFrame.point.z);
            goal.target_pose.pose.position.x = pointInMapFrame.point.x;
            goal.target_pose.pose.position.y = pointInMapFrame.point.y;
            goal.target_pose.pose.position.z = pointInMapFrame.point.z;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            ROS_INFO("go to the goal (in 'map' frame): x:[%f], y:[%f]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
            ac.sendGoal(goal);
            ac.waitForResult();
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("You have reached the goal!");
            else
                ROS_INFO("The base failed for some reason");
        }
        if(navGoal.stop) {
            // ROS_INFO("stop navigating! shoot enemies!");
            ac.cancelGoal();
            // ros::param::set("/real_world/is_aiming", navGoal.stop);
            // 如果正在自瞄，就取消当前正在追踪的目标
            // int navTerminate = 0;
            // ros::param::get("/real_world/is_aiming", navTerminate);
            // if(navTerminate) {
            // ac.cancelGoal();
            // }
        }

    }

    // void fakeNavGoalCallback(const sentry_communication::navGoalSimulation::ConstPtr& msg) {
    //     fake_navGoal.coo_x = msg.coo_x;
    //     fake_navGoal.coo_y = msg.coo_y;
    //     fake_navGoal.stop = msg.stop;
    //     fake_navGoal.color = msg.color;
    // }
    
    void timerUpdateCallback() {
        if(ros::Time::now() - statrTimeStamp < ros::Duration(2)) {
            return ;
        }
        // std::cout << "Timer Update Periodically: navState:[" << ac.getState().text_ << "]" << std::endl;
        // 定时发布导航状态信息
        std_msgs::String navStateStr;
        navStateStr.data = ac.getState().text_;
        pub_navState.publish(navStateStr);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener_mcu");
    TransmitNode_t TransmitNode;
    ros::spin(); // 进入消息循环
    return 0;
}

geometry_msgs::PoseWithCovarianceStamped fillInitialPose(const char* color_) {
    geometry_msgs::PoseWithCovarianceStamped initialPose;
    if(std::strcmp(color_, "BLUE") == 0) {   // color_ equals to "RED"
        initialPose.pose.pose.position.x = 0;
        initialPose.pose.pose.position.y = 0;
        initialPose.pose.pose.position.z = 0;
        tf::Quaternion quat;
        quat.setRPY(0,0,0);
        initialPose.pose.pose.orientation.w = quat.getW();
        initialPose.pose.pose.orientation.x = quat.getX();
        initialPose.pose.pose.orientation.y = quat.getY();
        initialPose.pose.pose.orientation.z = quat.getZ();
        initialPose.header.stamp = ros::Time::now();
        initialPose.header.frame_id = "map";
        ROS_INFO("initialPose: \n\t Position: x:[%f], y:[%f], z:[%f] \n\t Orientation: x:[%f], y:[%f], z:[%f], w:[%f] ", \
            initialPose.pose.pose.position.x, initialPose.pose.pose.position.y, initialPose.pose.pose.position.z, \
            initialPose.pose.pose.orientation.x, initialPose.pose.pose.orientation.y, initialPose.pose.pose.orientation.z, initialPose.pose.pose.orientation.w);
    }
    if(std::strcmp(color_, "RED") == 0) {
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
        initialPose.header.frame_id = "map";
        ROS_INFO("initialPose: \n\t Position: x:[%f], y:[%f], z:[%f] \n\t Orientation: x:[%f], y:[%f], z:[%f], w:[%f]" ,
            initialPose.pose.pose.position.x, initialPose.pose.pose.position.y, initialPose.pose.pose.position.z, \
            initialPose.pose.pose.orientation.x, initialPose.pose.pose.orientation.y, initialPose.pose.pose.orientation.z, initialPose.pose.pose.orientation.w);    
    }
    return initialPose;
}
