// #include "ros/ros.h"
#include <cstdint>

/**
 * @brief 云台手 -> 电控 -> move_base
 */
#pragma pack()
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
} navCommand_t;

#pragma pack(1)
typedef struct {            // 都使用朴素机器人坐标系,前x，左y,上z
    uint8_t frame_header;   // 帧头0θx20
    float x_speed;          // x方向速度
    float y_speed;          // y方向速度
    uint8_t is_reach;       // 是否到达
    float coo_x_current;    // 当前x坐标
    float coo_y_current;    // 当前y坐标
    uint32_t reserve_0: 8;  
    uint32_t reserve_1: 32;
    uint32_t reserve_2: 32;
    uint32_t reserve_3: 32;
    uint8_t frame_tail;     // 帧尾0x21
} navInfo_t;

#pragma pack()
