#include "ros/ros.h"

#pragma pack(1)
struct msg_cmd_t
{
    /* data */
    uint8_t markHead;
    float speedX;
    float speedY;
    float speedZ;
    uint8_t is_reach;
    uint8_t reverse_0;
    uint32_t reverse_1;
    uint32_t reverse_2;
    uint32_t reverse_3;
    uint32_t reverse_4;
    uint8_t markTail;
};

#pragma pack()