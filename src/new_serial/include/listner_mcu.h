#include "ros/ros.h"

#pragma pack(1)
typedef struct
{
    /* data */
    uint8_t head;
    float coo_x;
    float coo_y;
    uint16_t reverse_0;
    uint32_t reverse_1;
    uint32_t reverse_2;
    uint32_t reverse_3;
    uint32_t reverse_4;
    uint32_t reverse_5;
    uint8_t tail;
}msg_mcu_t;

#pragma pack()