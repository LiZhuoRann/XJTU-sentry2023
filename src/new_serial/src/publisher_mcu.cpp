#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
// #include "control/crc_m.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>
#include <thread>
#include <std_msgs/String.h>

#define WRITE_SIZE 14
#define READ_SIZE 64

serial::Serial sp;
std::string port_name;
ros::Publisher serial_pub;

int open_port() {
    // TODO: Add latch for Concurrency
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort(port_name);
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    int retry = 0;
    while (true) {
        retry++;
        try {
            sp.open();
            break;
        }
        catch (std::exception &e) {
            ROS_ERROR_STREAM("Unable to open port: " << port_name);
            // toggleLastCharacter(port_name);
            ros::Duration(0.1).sleep();
            if (retry > 4) {
                exit(0);
            }
        }
    }
    if (sp.isOpen()) {
        ROS_INFO_STREAM(port_name << " is Opened.");
    } else {
        return -1;
    }
    return 1;
}
/**
 * @brief 线程执行函数，监听串口数据并
 * @todo 想不通这个逻辑，如果休眠1ms时间到了，电控没有发来数据，串口会不会异常重启？如果数据错位能否自动修复？
 */
void readSerial()
{
    while (true)
    {
        try
        {
            if (sp.available())
            {
                std::string data_read = sp.read(sp.available());
                printf("\n[readSerial] readSerial: ");
                for (int i = 0; i < READ_SIZE; i++)
                {
                    printf("%x", data_read[i]);
                }
                std_msgs::String msg;
                msg.data = data_read;
                serial_pub.publish(msg);
            }
        }
        catch(const serial::IOException& e)
        {
            sp.close();
            open_port();
        }
        catch(const std::exception& e)
        {
            sp.close();
            open_port();
        }
        ros::Duration(0.001).sleep();   // 1kHz
        // 在每次操作后休眠0.001秒。这种模式常常用于读取和处理来自硬件设备（如传感器）的数据。这样可以确保主线程不会被阻塞，而且可以实时处理数据。
    }
    
}

/**
 * @brief Callback function for /cmd_vel_string topic
          发送
 * @param msg 指向接收到的消息的常量指针
 */
void cmdVelStringCallback(const std_msgs::String::ConstPtr& msg)
{
    try
    {
        printf("\n[Motor] data write: %x ", (unsigned char) msg->data.c_str()[0]);
        for (int i = 1; i < WRITE_SIZE ; i++)
        {
            printf("%x ", (unsigned char) msg->data.c_str()[i]);
        }
        // wirte date into serial port, start to transmit?
        sp.write(reinterpret_cast<const uint8_t *>(msg->data.c_str()), WRITE_SIZE);
    }
    catch (serial::IOException &e)
    {
        sp.close();
        open_port();
    }
    catch (std::exception &e)
    {
        sp.close();
        open_port();
    }
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "publisher_mcu");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("PortName", port_name, "/dev/ttyACM0");

    open_port();

    // Publish serial data to /serial_data topic
    serial_pub = nh.advertise<std_msgs::String>("/serial_data", 100);
    // Subscribe to /cmd_vel_string topic
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel_string", 100, cmdVelStringCallback);

    std::thread t1(readSerial);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_FATAL_STREAM("###### exit while ######");

    t1.join();
    sp.close();

    return 0;
}
