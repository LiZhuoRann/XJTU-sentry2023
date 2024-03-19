#include "ros/ros.h"

#pragma pack(1)

// communication protocal definition
typedef struct
{
    /* data */
    uint8_t head;   // ought to be 0x6f
    uint16_t hp;
    uint16_t bullet;
    uint16_t post;
    uint16_t time;
    uint8_t tail;   // ought to be 0x83
}msg_mcu_t;

#pragma pack()