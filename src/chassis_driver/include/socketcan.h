#pragma once
#include <linux/can.h>
#include <string>

#include "dbc_encoder_decoder.h"

int initializeSocketCAN(const std::string& interface);

// 新的原始数据接口 (用于 ZLAC8015D SDO 通信)
bool sendCANFrame(int socket_fd, uint32_t can_id, const uint8_t* data, uint8_t len);
int recvCANFrame(int socket_fd, uint32_t& can_id, uint8_t* data, uint8_t& dlc);

// BMS 唤醒帧使用 can_frame 结构体 (扩展帧)
bool sendCANFrame(int socket_fd, const struct can_frame& frame);
