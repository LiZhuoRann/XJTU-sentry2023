#include "ros/ros.h"

#pragma pack(1)

// 这里ROS msg 和 C struct 混用，怪怪的
struct msg_cmd_t
{
    /* data */
    uint8_t markHead;  // ought to be 0x5e
    float speedX;
    float speedY;
    float speedZ;
    uint8_t markTail;
};

#pragma pack()