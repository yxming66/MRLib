#pragma once

// C++ 电机协议的独立数据类型。
// 使用方式：上层通常不直接构造这些 raw 类型，而是通过 MotorT 的 Config、
// State 和 SetXXX 接口使用；本文件用于 traits、协议实现和测试之间共享协议字段。

#include <stdint.h>

namespace mr::motor {

enum class RmModule : uint8_t { M2006 = 0, M3508 = 1, Gm6020 = 2 };
enum class DmModule : uint8_t {
    J10010, J10010L, J10422P, J3507, J4310, J4310P, J4340, J4340P,
    J6006, J6248P, J8006, J8009, J8009P, H3510
};
enum class LzModule : uint8_t { Rso0, Rso1, Rso2, Rso3, Rso4, Rso5, Rso6 };
enum class LzControlMode : uint8_t { Motion = 1, Current = 2, Velocity = 3, Position = 4 };

// DM feedback D[0]: low nibble is motor ID, high nibble is ERR status.
enum class DmErrorStatus : uint8_t {
    Disabled = 0x0,
    Enabled = 0x1,
    OverVoltage = 0x8,
    UnderVoltage = 0x9,
    OverCurrent = 0xA,
    MosOverTemperature = 0xB,
    RotorOverTemperature = 0xC,
    CommunicationLost = 0xD,
    Overload = 0xE
};

struct RmFeedback {
    uint16_t angle = 0U;
    int16_t speed = 0;
    int16_t current = 0;
    uint8_t temperature = 0U;
    uint8_t error = 0U;
};

struct DmFeedback {
    uint16_t position = 0U;
    uint16_t velocity = 0U;
    uint16_t torque = 0U;
    uint8_t status = 0U;
    uint8_t mos_temperature = 0U;
    uint8_t rotor_temperature = 0U;
};

struct LzFeedback {
    uint16_t angle = 0U;
    uint16_t velocity = 0U;
    uint16_t torque = 0U;
    uint16_t temperature = 0U;
    uint8_t fault_bits = 0U;
    uint8_t state_bits = 0U;
};

}  // namespace mr::motor
