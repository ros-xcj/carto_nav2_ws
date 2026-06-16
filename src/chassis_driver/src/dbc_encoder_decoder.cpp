#include <dbc_encoder_decoder.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <algorithm>

dbc_encoder::dbc_encoder(){}
dbc_encoder::~dbc_encoder(){}

// ==========================================================
// ZLAC8015D 电机驱动 SDO/NMT 实现
// ==========================================================

void dbc_encoder::composeSDO_Read(uint16_t index, uint8_t sub_index, uint32_t& can_id, uint8_t data[8], uint8_t& dlc) {
    can_id = CAN_ID_SDO_TX;
    dlc = 8;
    std::memset(data, 0, 8);
    data[0] = 0x40; // Client Upload (Read Request)
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = sub_index;
}

bool dbc_encoder::parseSDO_Response(const uint8_t data[8], uint8_t dlc, double& left_rpm, double& right_rpm) {
    if (dlc < 8) return false;
    
    if (data[0] != 0x43) return false; // 只关心 4 字节响应

    uint16_t index = data[1] | (data[2] << 8);
    uint8_t sub_index = data[3];

    // 只处理 0x606C sub 03
    if (index != 0x606C || sub_index != 0x03) return false;

    // 解析数据 (低16位左轮，高16位右轮)
    int16_t raw_left = (int16_t)(data[4] | (data[5] << 8));
    int16_t raw_right = (int16_t)(data[6] | (data[7] << 8));

    // 计算实际转速 (单位 0.1 r/min)
    left_rpm = (double)raw_left / 10.0;
    right_rpm = (double)(-raw_right) / 10.0; // 右轮取反
    return true;
}

void dbc_encoder::composeSDO_Write4Byte(uint16_t index, uint8_t sub_index, int32_t value, uint32_t& can_id, uint8_t data[8], uint8_t& dlc) {
    can_id = CAN_ID_SDO_TX;
    dlc = 8;
    data[0] = 0x23; // 4 Byte Write
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = sub_index;
    data[4] = value & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = (value >> 16) & 0xFF;
    data[7] = (value >> 24) & 0xFF;
}

void dbc_encoder::composeSDO_Write2Byte(uint16_t index, uint8_t sub_index, int16_t value, uint32_t& can_id, uint8_t data[8], uint8_t& dlc) {
    can_id = CAN_ID_SDO_TX;
    dlc = 8;
    data[0] = 0x2B; // 2 Byte Write
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = sub_index;
    data[4] = value & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = 0x00;
    data[7] = 0x00;
}

void dbc_encoder::composeSDO_Write1Byte(uint16_t index, uint8_t sub_index, int8_t value, uint32_t& can_id, uint8_t data[8], uint8_t& dlc) {
    can_id = CAN_ID_SDO_TX;
    dlc = 8;
    data[0] = 0x2F; // 1 Byte Write
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = sub_index;
    data[4] = value & 0xFF;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
}

void dbc_encoder::composeNMT(uint8_t command, uint8_t node_id, uint32_t& can_id, uint8_t data[8], uint8_t& dlc) {
    can_id = CAN_ID_NMT;
    dlc = 2;
    data[0] = command;
    data[1] = node_id;
}

void dbc_encoder::composeReserve(uint32_t& can_id, uint8_t data[8], uint8_t& dlc) {
    can_id = CAN_ID_NMT;
    dlc = 8;
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x80;
    data[7] = 0x00;
}

// ==========================================================
// BMS 协议实现 (保留)
// ==========================================================

// 编码 BMS 唤醒/控制帧 (0x0400FF80)
void dbc_encoder::encodeBMSControl(struct can_frame& frame) {
    frame.can_id = 0x0400FF80 | CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;
    std::memset(frame.data, 0, sizeof(frame.data));
}

void dbc_encoder::parseFrame(const struct can_frame& frame, BMS_Data& bms_data) {
    uint32_t id = frame.can_id & CAN_EFF_MASK;
    uint32_t pg_id = id & 0xFFFFFF00; 

    switch (pg_id) {
        case 0x04028000:
            parseTotalInfo0(frame, bms_data.info0);
            break;
        case 0x04038000:
            parseTotalInfo1(frame, bms_data.info1);
            break;
        case 0x04048000:
        case 0x04058000:
            parseStats(frame, bms_data.stats);
            break;
        case 0x04068000:
            parseStatus0(frame, bms_data.status0);
            break;
        case 0x04078000:
            parseStatus1(frame, bms_data.status1);
            break;
        case 0x04088000:
            parseStatus2(frame, bms_data.status2);
            break;
        case 0x040C8000:
            parseTime(frame, bms_data.time);
            break;
        case 0x040E8000:
            parseFaultInfo(frame, bms_data.fault_data);
            break;
        case 0x04008000:
            parseCellVoltage(frame);
            break;
        case 0x04018000:
            parseCellTemp(frame); 
            break;
        default:
            break;
    }
}

void dbc_encoder::parseCellVoltage(const struct can_frame& frame) {
    // 暂未实现单体电压解析
}

void dbc_encoder::parseCellTemp(const struct can_frame& frame) {
    // 暂未实现单体温度解析
}

void dbc_encoder::parseTotalInfo0(const struct can_frame& frame, BMS_TotalInfo0& info) {
    uint16_t sum_v = (frame.data[0] << 8) | frame.data[1];
    info.sum_voltage = sum_v * 0.1;
    
    uint16_t curr_raw = (frame.data[2] << 8) | frame.data[3];
    info.current = (curr_raw * 0.1) - 3000.0; 
    
    uint16_t soc_raw = (frame.data[4] << 8) | frame.data[5];
    info.soc = soc_raw * 0.1;
    
    info.life = frame.data[6];
}

void dbc_encoder::parseTotalInfo1(const struct can_frame& frame, BMS_TotalInfo1& info) {
    info.power = (int16_t)((frame.data[0] << 8) | frame.data[1]);
    info.total_energy = (int16_t)((frame.data[2] << 8) | frame.data[3]);
    info.mos_temp = (int16_t)frame.data[4] - 40;
    info.board_temp = (int16_t)frame.data[5] - 40;
    info.heat_temp = (int16_t)frame.data[6] - 40;
    info.heat_current = frame.data[7];
}

void dbc_encoder::parseStats(const struct can_frame& frame, BMS_Stats& stats) {
    uint32_t id = frame.can_id & 0xFFFFFF00;
    if (id == 0x04048000) {
        stats.max_v = (frame.data[0] << 8) | frame.data[1];
        stats.max_v_no = frame.data[2];
        stats.min_v = (frame.data[3] << 8) | frame.data[4];
        stats.min_v_no = frame.data[5];
        stats.diff_v = (frame.data[6] << 8) | frame.data[7];
    } else if (id == 0x04058000) {
        stats.max_t = (int16_t)frame.data[0] - 40;
        stats.max_t_no = frame.data[1];
        stats.min_t = (int16_t)frame.data[2] - 40;
        stats.min_t_no = frame.data[3];
        stats.diff_t = (int16_t)frame.data[4];
    }
}

void dbc_encoder::parseStatus0(const struct can_frame& frame, BMS_Status0& status) {
    status.chg_mos_state = frame.data[0];
    status.dchg_mos_state = frame.data[1];
    status.pre_mos_state = frame.data[2];
    status.heat_mos_state = frame.data[3];
    status.fan_mos_state = frame.data[4];
    status.do_state = frame.data[5];
    status.di_state = frame.data[6];
}

void dbc_encoder::parseStatus1(const struct can_frame& frame, BMS_Status1& status) {
    status.bat_state = frame.data[0];
    status.chg_detect = frame.data[1];
    status.load_detect = frame.data[2];
}

void dbc_encoder::parseStatus2(const struct can_frame& frame, BMS_Status2& status) {
    status.cell_number = frame.data[0];
    status.ntc_number = frame.data[1];
    status.remain_capacity = (frame.data[2] << 24) | (frame.data[3] << 16) | (frame.data[4] << 8) | frame.data[5]; 
    status.cycle_time = (frame.data[6] << 8) | frame.data[7];
}

void dbc_encoder::parseTime(const struct can_frame& frame, BMS_Time& time) {
    time.year = 2000 + frame.data[0];
    time.month = frame.data[1];
    time.day = frame.data[2];
    time.hour = frame.data[3];
    time.minute = frame.data[4];
    time.second = frame.data[5];
}

void dbc_encoder::parseFaultInfo(const struct can_frame& frame, BMS_FaultData& fault) {
    uint8_t page_no = frame.data[0];
    if (page_no == 1) {
        std::memcpy(fault.page1, frame.data, 8);
    } else if (page_no == 2) {
        std::memcpy(fault.page2, frame.data, 8);
    }
}

std::string BMS_FaultData::getFaultDescription() const {
    std::stringstream ss;
    bool fault_found = false;

    auto append = [&](const char* msg) {
        if (fault_found) ss << "; ";
        ss << msg;
        fault_found = true;
    };
    
    // Page 1
    if ((page1[1] & 0x07) > 0) append("充电低温告警");
    if ((page1[1] & 0x38) > 0) append("放电高温告警");
    if (page1[1] & 0x40) append("充电MOS过温");
    if (page1[1] & 0x80) append("充电MOS温度传感器故障");

    if ((page1[2] & 0x07) > 0) append("放电低温告警");
    if ((page1[2] & 0x38) > 0) append("压差过大告警");
    if (page1[2] & 0x40) append("放电MOS过温");
    if (page1[2] & 0x80) append("放电MOS温度传感器故障");

    if ((page1[3] & 0x07) > 0) append("总压过高告警");
    if ((page1[3] & 0x38) > 0) append("总压过低告警");
    if (page1[3] & 0x40) append("短路保护");
    if (page1[3] & 0x80) append("高压禁止放电");

    if ((page1[4] & 0x07) > 0) append("充电过流告警");
    if ((page1[4] & 0x38) > 0) append("放电过流告警");
    if (page1[4] & 0x40) append("低压禁止充电");
    if (page1[4] & 0x80) append("并机通信失败");

    if ((page1[5] & 0x07) > 0) append("SOC过低告警");
    if ((page1[5] & 0x38) > 0) append("SOH过低告警");

    // Page 2
    if (page2[1] & 0x01) append("AFE芯片故障");
    if (page2[1] & 0x02) append("AFE通信故障");
    if (page2[1] & 0x04) append("AFE采样故障");
    if (page2[1] & 0x08) append("电压检测故障");
    if (page2[1] & 0x10) append("电压采集线掉线");
    if (page2[1] & 0x20) append("总压检测故障");
    if (page2[1] & 0x40) append("电流检测故障");
    if (page2[1] & 0x80) append("温度检测故障");

    if (page2[2] & 0x01) append("温度采集线掉线");
    if (page2[2] & 0x02) append("EEPROM存储故障");
    if (page2[2] & 0x04) append("Flash存储故障");
    if (page2[2] & 0x08) append("RTC时钟故障");
    if (page2[2] & 0x10) append("充电MOS故障");
    if (page2[2] & 0x20) append("放电MOS故障");
    if (page2[2] & 0x40) append("预充MOS故障");
    if (page2[2] & 0x80) append("预充失败");

    if (!fault_found) {
        return "systerm normal";
    }
    return ss.str();
}

uint64_t BMS_FaultData::getFaultCode() const {
    uint64_t code = 0;
    
    if ((page1[1] & 0x07) > 0) code |= (1ULL << 0);
    if ((page1[1] & 0x38) > 0) code |= (1ULL << 1);
    if (page1[1] & 0x40) code |= (1ULL << 2);
    if (page1[1] & 0x80) code |= (1ULL << 3);

    if ((page1[2] & 0x07) > 0) code |= (1ULL << 4);
    if ((page1[2] & 0x38) > 0) code |= (1ULL << 5);
    if (page1[2] & 0x40) code |= (1ULL << 6);
    if (page1[2] & 0x80) code |= (1ULL << 7);

    if ((page1[3] & 0x07) > 0) code |= (1ULL << 8);
    if ((page1[3] & 0x38) > 0) code |= (1ULL << 9);
    if (page1[3] & 0x40) code |= (1ULL << 10);
    if (page1[3] & 0x80) code |= (1ULL << 11);

    if ((page1[4] & 0x07) > 0) code |= (1ULL << 12);
    if ((page1[4] & 0x38) > 0) code |= (1ULL << 13);
    if (page1[4] & 0x40) code |= (1ULL << 14);
    if (page1[4] & 0x80) code |= (1ULL << 15);
    
    if ((page1[5] & 0x07) > 0) code |= (1ULL << 16);
    if ((page1[5] & 0x38) > 0) code |= (1ULL << 17);

    if (page2[1] & 0x01) code |= (1ULL << 18);
    if (page2[1] & 0x02) code |= (1ULL << 19);
    if (page2[1] & 0x04) code |= (1ULL << 20);
    if (page2[1] & 0x08) code |= (1ULL << 21);
    if (page2[1] & 0x10) code |= (1ULL << 22);
    if (page2[1] & 0x20) code |= (1ULL << 23);
    if (page2[1] & 0x40) code |= (1ULL << 24);
    if (page2[1] & 0x80) code |= (1ULL << 25);

    if (page2[2] & 0x01) code |= (1ULL << 26);
    if (page2[2] & 0x02) code |= (1ULL << 27);
    if (page2[2] & 0x04) code |= (1ULL << 28);
    if (page2[2] & 0x08) code |= (1ULL << 29);
    if (page2[2] & 0x10) code |= (1ULL << 30);
    if (page2[2] & 0x20) code |= (1ULL << 31);
    if (page2[2] & 0x40) code |= (1ULL << 32);
    if (page2[2] & 0x80) code |= (1ULL << 33);

    return code;
}
