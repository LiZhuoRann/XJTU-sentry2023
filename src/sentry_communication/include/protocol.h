// #include "ros/ros.h"
#include <cstdint>

#pragma pack(1)
typedef struct {            // 都使用朴素机器人坐标系,前x,左y,上z
    uint8_t frame_header;   // 帧头 0x20
    float coo_x;            // x 坐标
    float coo_y;            // y 坐标
    uint8_t stop;           // 停止
    uint8_t color;          // 机器人颜色（0=RED, 1=BLUE）
    uint32_t reverse_1: 32;
    uint32_t reverse_2: 32;
    uint32_t reverse_3: 32;
    uint32_t reverse_4: 32;
    uint32_t reverse_5: 32;
    uint8_t frame_tail;     //帧尾 0x21
} navCommand_t;
#pragma pack()

#pragma pack(1) 
typedef struct {            // 都使用朴素机器人坐标系,前x,左y,上z
    uint8_t frame_header;   // 帧头 0x20
    float x_speed;          // x 方向速度
    float y_speed;          // y 方向速度
    uint8_t is_reach;       // 是否到达 TODO: 可以改成 move_base/result 提供更多的反馈信息
    float coo_x_current;    // 当前 x 坐标
    float coo_y_current;    // 当前 y 坐标
    uint8_t navState;
    uint32_t reserve_1: 32;
    uint32_t reserve_2: 32;
    uint32_t reserve_3: 32;
    uint8_t frame_tail;     //帧尾 0x21
} navInfo_t;
#pragma pack()
