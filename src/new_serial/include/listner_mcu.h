#include "ros/ros.h"

#pragma pack(1)
typedef struct
{
    /* data */
    uint8_t head;
    uint16_t hp;
    uint16_t bullet;
    uint16_t post;
    uint16_t time;
    uint8_t tail;
}msg_mcu_t;

#pragma pack()