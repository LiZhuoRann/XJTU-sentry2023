#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/duration.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <boost/bind.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <thread>

namespace communicationNode  {
/* ------------ 通信协议 -------------- */
// 电控下位机发来的**导航目的地**指令
#pragma pack(1)
typedef struct {            // 都使用朴素机器人坐标系,前x,左y,上z
    uint8_t frame_header;   // 帧头 0x20
    float coo_x;            // x坐标
    float coo_y;            // y坐标
    uint8_t stop;           // 停止
    uint32_t reverse_0: 8;
    uint32_t reverse_1: 32;
    uint32_t reverse_2: 32;
    uint32_t reverse_3: 32;
    uint32_t reverse_4: 32;
    uint32_t reverse_5: 32;
    uint8_t frame_tail;     // 帧尾 0x21
} navGoalRecv_t;
#pragma pack() 

// 发给电控下位机的**导航状态信息**
#pragma pack(1)
typedef struct {            //都使用朴素机器人坐标系,前x,左y,上z
    uint8_t frame_header;   //帧头 0x20
    float x_speed;          //x方向速度
    float y_speed;          //y方向速度
    uint8_t is_reach;       //是否到达
    float coo_x_current;    //当前x坐标
    float coo_y_current;    //当前y坐标
    uint32_t reserve_0: 8;
    uint32_t reserve_1: 32;
    uint32_t reserve_2: 32;
    uint32_t reserve_3: 32;
    uint8_t frame_tail;     //帧尾 0x21
} navInfoTrans_t;

#pragma pack()

/*------------------ 串口配置 ----------------*/
class serialConfig {
public:
    int baudRate;
    std::string portName;
    serial::Timeout to;
public:
    serialConfig();
    serialConfig(std::uint32_t baudRate, std::string portName) : baudRate(baudRate), portName(portName), to(100) {};
};

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

/*------------------ 通信节点 ----------------*/
static ros::Subscriber sub_cmdVel;
static ros::Subscriber sub_odom;
static ros::Publisher  pub_movebaseGoal;
static navInfoTrans_t navInfo;
static navGoalRecv_t navGoal;
static ros::NodeHandle nh;
static ros::Timer timer;
static serial::Serial sp;
static serialConfig sc;
static std::thread readThread;

void communicationNodeInit();
bool open_port(serial::Serial& sp, const serialConfig& sc) ;
void readSerial();
template<class T>
inline void writeSerial(serial::Serial& sp, const serialConfig& sc, T data);
void subCmdVel_cb(const geometry_msgs::Twist::ConstPtr &msg);
void subOdom_cb(const nav_msgs::Odometry::ConstPtr &msg);
void timer_cb(const ros::TimerEvent&);
void strategyHandler();

void communicationNodeInit() {
    nh.param<std::string>("seraial/portName", sc.portName, "/dev/ttyACM0");
    nh.param<int>("serial/baudRate", sc.baudRate, 115200);
    while (!open_port(sp, sc)) {
        ros::Duration(0,1).sleep();
        ROS_INFO_STREAM("Serial Port Open Failed, trying again...");
    }
    ROS_INFO_STREAM("Serial Port Open Successed!");
    // 机器人速度和位置信息的订阅者
    sub_cmdVel = nh.subscribe("/cmd_vel", 100, &communicationNode::subCmdVel_cb);
    sub_odom = nh.subscribe("/Odom", 100, &communicationNode::subOdom_cb);
    // 定时发给电控 navInfo 
    timer = nh.createTimer(ros::Duration(0.1), &communicationNode::timer_cb);
    // 读取串口线程
    readThread = std::thread(readSerial);
    readThread.join();
}
// 回调函数
void subCmdVel_cb(const geometry_msgs::Twist::ConstPtr &msg) {
    navInfo.x_speed = msg->linear.x;
    navInfo.y_speed = msg->linear.y;
    ROS_INFO("Received /cmd_vel: x_speed = %f, y_speed = %f", navInfo.x_speed, navInfo.y_speed);
}
// 回调函数
void subOdom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    navInfo.coo_x_current = msg->pose.pose.position.x;
    navInfo.coo_y_current = msg->pose.pose.position.y;
    ROS_INFO("Received /Odom: x_coo_current = %f, x_coo_current = %f", navInfo.coo_x_current, navInfo.coo_y_current);
}
// 回调函数
void timer_cb(const ros::TimerEvent&) {
    navInfo.frame_header = 0x20;
    navInfo.frame_tail = 0x21;
    // 发送给下位机
    writeSerial(sp, sc, navInfo);
}
bool open_port(serial::Serial& sp, const serialConfig& sc) {
    // TODO: Add latch for Concurrency
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort(sc.portName);
    sp.setBaudrate(sc.baudRate);
    sp.setTimeout(to);
    std::uint32_t retry = 0;

    while (true) {
        retry++;
        try {
            sp.open();
            break;
        }
        catch (std::exception &e) {
            ROS_ERROR_STREAM("Unable to open port" + sc.portName);
            // toggleLastCharacter(port_name);
            ros::Duration(0.1).sleep();
            if (retry > 4) {
                exit(0);
            }
        }
    }
    if (sp.isOpen()) {
        ROS_INFO_STREAM(sc.portName + " is Opened.");
    } else {
        return -1;
    }
    return 1;
}
template<class T>
inline void writeSerial(serial::Serial& sp, const serialConfig& sc, T data)
{
    try {
        // wirte date into serial port
        sp.write(reinterpret_cast<const unsigned char*>(&data), sizeof(data));
    }
    catch (serial::IOException &e)
    {
        sp.close();
        open_port(sp, sc);
    }
    catch (std::exception &e)
    {
        sp.close();
        open_port(sp, sc);
    }
}

/**
* @brief 监听串口数据
* @todo 想不通这个逻辑，如果休眠1ms时间到了，电控没有发来数据，串口会不会异常重启？
        如果数据错位能否自动修复？
*/
void readSerial()
{
    while (true)
    {
        try
        {
            if (sp.available())   /*! Return the number of characters in the buffer. */
            {
                std::string data_read = sp.read(sp.available());
                ROS_INFO_STREAM("\n[readSerial] readSerial: " + data_read);
                auto navGoalStr = const_cast<char*>(data_read.data());
                navGoalRecv_t* goal = reinterpret_cast<navGoalRecv_t*>(navGoalStr);
                // TODO: 这里完全是瞎写的，大概率不能用
                navGoal.coo_x = goal->coo_x;
                navGoal.coo_y = goal->coo_y;
                // std::memcpy(&navGoal, reinterpret_cast<navGoalRecv_t*>(navGoalStr), sizeof(navGoal));
                strategyHandler();
            }
        }
        catch(const serial::IOException& e)
        {
            sp.close();
            open_port(sp, sc);
        }
        catch(const std::exception& e)
        {
            sp.close();
            open_port(sp, sc);
        }
        ros::Duration(0.001).sleep();   // 1kHz
        // 在每次操作后休眠0.001秒。这种模式常常用于读取和处理来自硬件设备（如传感器）的数据。这样可以确保主线程不会被阻塞，而且可以实时处理数据。
    }
}
// 回调函数
void strategyHandler() {
    // TODO" (coo_x, coo_y) 到 2dmap 的映射
    ;

    // 告诉 action client 我们要连接到 move_base server
    static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    static bool initFlag = true;
    if(initFlag) {
    // 等待 server 变为可用
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        initFlag = false;
    } 
    
    move_base_msgs::MoveBaseGoal goal;
    // 设置目标点的 frame
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    // 设置目标点的位置和方向
    goal.target_pose.pose.position.x = navGoal.coo_x;
    goal.target_pose.pose.position.x = navGoal.coo_y;

    ROS_INFO("\n[move_base]: Sending goal");
    
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The base moved to the goal position");
    else
        ROS_INFO("The base failed to move to the goal position");
}

}

