#include "socketcan.h"
#include <iostream>
#include <iomanip>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>

int initializeSocketCAN(const std::string& interface) {
    int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        perror("Socket creation failed");
        return -1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        close(socket_fd);
        return -1;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        close(socket_fd);
        return -1;
    }

    // Set receive buffer size
    int receive_buffer_size = 256 * 1024;
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer_size, sizeof(receive_buffer_size));
    
    // Set non-blocking
    int flags = fcntl(socket_fd, F_GETFL, 0);
    fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
    
    std::cout << "SocketCAN init success on " << interface << std::endl;
    return socket_fd;
}

// 新接口：使用原始 id + data 发送 (用于 ZLAC8015D SDO 通信)
bool sendCANFrame(int socket_fd, uint32_t can_id, const uint8_t* data, uint8_t len) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = len;
    std::memset(frame.data, 0, 8);
    if (len > 0) {
        std::memcpy(frame.data, data, len);
    }

    if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame)) {       
        perror("Write failed");
        return false;
    }
    return true;
}

// 保留：使用 can_frame 结构体发送 (用于 BMS 扩展帧)
bool sendCANFrame(int socket_fd, const struct can_frame& frame) {
    if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame)) {       
        perror("Write failed");
        return false;
    }
    return true;
}

// 非阻塞接收
int recvCANFrame(int socket_fd, uint32_t& can_id, uint8_t* data, uint8_t& dlc) {
    struct can_frame frame;
    int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return 0; // No data available
        } else {
            perror("recv failed");
            return -1;
        }
    } else if (nbytes == sizeof(struct can_frame)) {
        if (frame.can_id & CAN_ERR_FLAG) return 0;
        if (frame.can_id & CAN_RTR_FLAG) return 0;
        
        can_id = frame.can_id; // 保留 EFF_FLAG 用于 BMS 扩展帧判断
        dlc = frame.can_dlc;
        std::memset(data, 0, 8);
        if (dlc > 0) {
            std::memcpy(data, frame.data, dlc);
        }
        return 1;
    }
    return 0;
}