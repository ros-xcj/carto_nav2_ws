#pragma once
#include <linux/can.h>
#include <cstring>
#include <cstdint>
#include <string>
#include <sstream>

// ==========================================================
// ZLAC8015D CANopen 常量定义
// ==========================================================
#define ZLAC_NODE_ID 0x01
#define CAN_ID_TPDO1 (0x180 + ZLAC_NODE_ID) // 发送PDO1 (从站->主站)
#define CAN_ID_RPDO1 (0x200 + ZLAC_NODE_ID) // 接收PDO1 (默认映射 ControlWord)
#define CAN_ID_RPDO2 (0x300 + ZLAC_NODE_ID) // 接收PDO2 (映射 TargetVelocity)
#define CAN_ID_SDO_TX (0x600 + ZLAC_NODE_ID) // Master -> Slave (SDO 请求)
#define CAN_ID_SDO_RX (0x580 + ZLAC_NODE_ID) // Slave -> Master (SDO 响应)
#define CAN_ID_NMT    0x000                  // NMT 网络管理 ID

// NMT 网络管理命令
#define NMT_START_REMOTE_NODE 0x01
#define NMT_STOP_REMOTE_NODE  0x02
#define NMT_ENTER_PRE_OP      0x80
#define NMT_RESET_NODE        0x81
#define NMT_RESET_COMM        0x82

// ==========================================================
// BMS 结构体定义 (保留)
// ==========================================================

// BMS 总信息 0 结构体
struct BMS_TotalInfo0 {
    double sum_voltage;    // 总电压 (0.1V)
    double current;        // 电流 (0.1A, 偏移 -30000)
    double soc;            // 剩余电量 (0.1%)
    uint8_t life;          // 寿命 (0-255)
};

// BMS 总信息 1 结构体
struct BMS_TotalInfo1 {
    int32_t power;         // 功率 (1W)
    int32_t total_energy;  // 总能量 (1WH)
    int16_t mos_temp;      // MOS 温度 (1C, 偏移 -40)
    int16_t board_temp;    // 板载温度 (1C, 偏移 -40)
    int16_t heat_temp;     // 加热温度 (1C, 偏移 -40)
    double heat_current;   // 加热电流 (1A)
};

// BMS 统计信息结构体
struct BMS_Stats {
    uint16_t max_v;        // 最高单体电压 (1mV)
    uint8_t max_v_no;      // 最高单体电压编号
    uint16_t min_v;        // 最低单体电压 (1mV)
    uint8_t min_v_no;      // 最低单体电压编号
    uint16_t diff_v;       // 压差 (1mV)
    int16_t max_t;         // 最高温度 (1C, 偏移 -40)
    uint8_t max_t_no;      // 最高温度编号
    int16_t min_t;         // 最低温度 (1C)
    uint8_t min_t_no;      // 最低温度编号
    int16_t diff_t;        // 温差 (1C)
};

// BMS 状态信息 0 结构体
struct BMS_Status0 {
    uint8_t chg_mos_state;
    uint8_t dchg_mos_state;
    uint8_t pre_mos_state;
    uint8_t heat_mos_state;
    uint8_t fan_mos_state;
    uint8_t do_state;
    uint8_t di_state;
};

// BMS 状态信息 1 结构体
struct BMS_Status1 {
    uint8_t bat_state;
    uint8_t chg_detect;
    uint8_t load_detect;
};

// BMS 状态信息 2 结构体
struct BMS_Status2 {
    uint8_t cell_number;
    uint8_t ntc_number;
    uint32_t remain_capacity; // mAH
    uint16_t cycle_time;
};

// BMS 时间结构体
struct BMS_Time {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

// BMS 故障信息结构体
struct BMS_FaultData {
    uint8_t page1[8];
    uint8_t page2[8];
    
    BMS_FaultData() {
        std::memset(page1, 0, sizeof(page1));
        std::memset(page2, 0, sizeof(page2));
    }

    std::string getFaultDescription() const;
    uint64_t getFaultCode() const;
};

// BMS 聚合数据结构体
struct BMS_Data {
    BMS_TotalInfo0 info0;
    BMS_TotalInfo1 info1;
    BMS_Stats stats;
    BMS_Status0 status0;
    BMS_Status1 status1;
    BMS_Status2 status2;
    BMS_Time time;
    BMS_FaultData fault_data;

    BMS_Data() {
        std::memset(&info0, 0, sizeof(info0));
        std::memset(&info1, 0, sizeof(info1));
        std::memset(&stats, 0, sizeof(stats));
        std::memset(&status0, 0, sizeof(status0));
        std::memset(&status1, 0, sizeof(status1));
        std::memset(&status2, 0, sizeof(status2));
        std::memset(&time, 0, sizeof(time));
    }
};


class dbc_encoder{
    public:
        dbc_encoder();
        ~dbc_encoder();

        // ==========================================================
        // ZLAC8015D 电机驱动 SDO/NMT 接口 (新增)
        // ==========================================================
        
        // SDO 读取请求 (CMD=0x40)
        void composeSDO_Read(uint16_t index, uint8_t sub_index, uint32_t& can_id, uint8_t data[8], uint8_t& dlc);

        // 解析 SDO 响应 — 返回 true 表示是速度数据 (Index=0x606C)
        bool parseSDO_Response(const uint8_t data[8], uint8_t dlc, double& left_rpm, double& right_rpm);

        // SDO 写入辅助函数
        void composeSDO_Write4Byte(uint16_t index, uint8_t sub_index, int32_t value, uint32_t& can_id, uint8_t data[8], uint8_t& dlc);
        void composeSDO_Write2Byte(uint16_t index, uint8_t sub_index, int16_t value, uint32_t& can_id, uint8_t data[8], uint8_t& dlc);
        void composeSDO_Write1Byte(uint16_t index, uint8_t sub_index, int8_t value, uint32_t& can_id, uint8_t data[8], uint8_t& dlc);
        
        // NMT 命令封装
        void composeNMT(uint8_t command, uint8_t node_id, uint32_t& can_id, uint8_t data[8], uint8_t& dlc);

        void composeHeatBeat(uint32_t& can_id, uint8_t data[8], uint8_t& dlc);
        void composeReserve(uint32_t& can_id, uint8_t data[8], uint8_t& dlc);

        // ==========================================================
        // BMS 接口 (保留)
        // ==========================================================
        
        // 生成 BMS 唤醒报文 (0x0400FF80)
        void encodeBMSControl(struct can_frame& frame);

        // 通用帧解析入口
        void parseFrame(const struct can_frame& frame, BMS_Data& bms_data);
        
        // 辅助解析方法
        void parseCellVoltage(const struct can_frame& frame);
        void parseCellTemp(const struct can_frame& frame);
        void parseTotalInfo0(const struct can_frame& frame, BMS_TotalInfo0& info);
        void parseTotalInfo1(const struct can_frame& frame, BMS_TotalInfo1& info);
        void parseStats(const struct can_frame& frame, BMS_Stats& stats);
        void parseStatus0(const struct can_frame& frame, BMS_Status0& status);
        void parseStatus1(const struct can_frame& frame, BMS_Status1& status);
        void parseStatus2(const struct can_frame& frame, BMS_Status2& status);
        void parseTime(const struct can_frame& frame, BMS_Time& time);
        void parseFaultInfo(const struct can_frame& frame, BMS_FaultData& fault);
};
