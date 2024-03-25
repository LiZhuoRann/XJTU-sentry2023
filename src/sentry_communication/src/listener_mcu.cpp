#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "protocol.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <ros/console.h>
#include "sentry_communication/navGoal.h"

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
navCommand_t fake_navGoal = {
    .frame_header = 0x20,
    .frame_tail   = 0x21,
};

double param_x;
double param_y;
int param_stop;
int param_color;
void fakeNavGoalCallback(const sentry_communication::navGoal::ConstPtr& msg);

geometry_msgs::PoseWithCovarianceStamped fillInitialPose(const char* color_);

class TransmitNode_t {
private:
    ros::NodeHandle nh;
    ros::Publisher pub_initialPose;
    ros::Publisher pub_navGoal;
    ros::Subscriber sub_serial;
    ros::Subscriber sub_fakeGoal;
    bool debug_mode = false;
    ros::Time statrTimeStamp;

public:
    // 构造函数中使用初始化列表，来初始化成员类 ac
    TransmitNode_t() {
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
            fake_navGoal.coo_x = param_x;
            fake_navGoal.coo_y = param_y;
            fake_navGoal.stop = param_stop;
            fake_navGoal.color = param_color;
            ROS_INFO("debug_mode (as integer): [%d], param_x:[%f], fake_navGoal.coo_x:[%f]",debug_mode, param_x, fake_navGoal.coo_x);
        }
        // 创建 Subscriber 对象，订阅 serial_data 主题
        sub_serial = nh.subscribe<std_msgs::String>("/serial_data", 100, &TransmitNode_t::serialDataCallback, this);
        sub_fakeGoal = nh.subscribe<sentry_communication::navGoal>("/fakeNavGoal", 1, fakeNavGoalCallback);
        
        // 根据机器人颜色不同，发布 /initialPose 话题
        pub_initialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1,true);
        pub_navGoal = nh.advertise<sentry_communication::navGoal>("/navGoal", 1, true);
    }

    // 收到串口数据后的回调函数
    void serialDataCallback(const std_msgs::String::ConstPtr &msg) {
        if(ros::Time::now() - statrTimeStamp < ros::Duration(2)) {
            return ;
        }
        // 处理串口数据，这里只是假设打印接收到的数据
        navGoal = deserialize<navCommand_t>(msg->data.c_str());
        // debug 模式，navGoal 由参数文件或自定义消息给定
        if(debug_mode) {
            ROS_INFO("fakeNavGoal: x:[%f], y:[%f], stop:[%d], color:[%d]", \
                fake_navGoal.coo_x, fake_navGoal.coo_y, fake_navGoal.stop, fake_navGoal.color);
            navGoal = fake_navGoal;
        }

        // 检查帧头帧尾
        if ((navGoal.frame_header != 0x20) || (navGoal.frame_tail != 0x21)){
            return;
        }

        // 初始化出生点，发布 /initialpose 和 map2camera_init 的静态tf变换
        static bool color_initialized = false;
        if (!color_initialized && navGoal.color == 1) {    // 1=RED
            ROS_INFO("*************** Robot Color is RED! ***************");
            color_initialized = true;
            pub_initialPose.publish(fillInitialPose("RED"));
        } 
        else if (!color_initialized && navGoal.color == 2) {    // 2=BLUE
            ROS_INFO("*************** Robot Color is BLUE! ***************");
            color_initialized = true;
            pub_initialPose.publish(fillInitialPose("BLUE"));
        } 
        
        static float last_x_cmd = 1;
        static float last_y_cmd = 1;
        if(last_x_cmd != navGoal.coo_x || last_y_cmd != navGoal.coo_y) {
            last_x_cmd = navGoal.coo_x;
            last_y_cmd = navGoal.coo_y;
            // 发布 /navGoal 话题
            sentry_communication::navGoal goal;
            goal.coo_x = navGoal.coo_x; 
            goal.coo_y = navGoal.coo_y; 
            goal.color = navGoal.color; 
            goal.stop = navGoal.stop; 
            pub_navGoal.publish(goal);
            ROS_INFO("go to the goal (in 'realWorld' frame): x:[%f], y:[%f]", goal.coo_x, goal.coo_y);
        }
        // ros::param::set("/navigatiom/coo_x", navGoal.coo_x);
        // ros::param::set("/navigation/coo_y", navGoal.coo_y);
        ros::param::set("/real_world/is_aiming", navGoal.stop);
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

 // void fakeNavGoalCallback(const sentry_communication::navGoalSimulation::ConstPtr& msg) {
void fakeNavGoalCallback(const sentry_communication::navGoal::ConstPtr& msg) {
    fake_navGoal.coo_x = msg->coo_x;
    fake_navGoal.coo_y = msg->coo_y;
    fake_navGoal.stop = msg->stop;
    fake_navGoal.color = msg->color;
}
