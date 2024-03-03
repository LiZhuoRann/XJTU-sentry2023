#include "ros/ros.h"

#pragma pack(1)
struct msg_cmd_t
{
    /* data */
    uint8_t markHead;
    float speedX;
    float speedY;
    float speedZ;
    uint8_t markTail;
};

#pragma pack()