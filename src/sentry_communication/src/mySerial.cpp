/*****************************************/
/**** WARNING：THIS FILE IS ABANDONED！****/
/*****************************************/

#include "mySerial.hpp" 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <string>

using namespace communicationNode;

int main(int argc, char** argv) {
    ros::init(argc, argv, "new_serial");
    communicationNodeInit();
    
    ros::spin();
}
