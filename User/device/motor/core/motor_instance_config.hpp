#pragma once

// 单个电机的总线和地址配置。使用方式：按 MotorKind 构造对应 Config，填入
// CAN、反馈/控制 ID 和 reverse，再传给 MotorFactory::Create 或 MotorT 构造函数。

// 定义按 MotorKind 分类的实例配置结构。
// 用于在通用 motor 配置与底层厂商参数结构之间做集中转换。

#include <stdint.h>

#include "device/motor/bus/can_transport.hpp"
#include "device/motor/core/motor_kind.hpp"
#include "device/motor/protocol/protocol_types.hpp"

namespace mr::motor {
class RmCommandBus;

template <MotorKind Kind>
struct MotorInstanceConfig;

template <>
struct MotorInstanceConfig<MotorKind::RM> {
    CanPort* port = nullptr;
    RmCommandBus* commands = nullptr;
    // RM feedback CAN identifier (M2006/M3508: 0x201-0x208;
    // GM6020: 0x205-0x20b). The command group/slot is derived with the model.
    uint16_t id = 0;
    bool reverse = false;
    uint32_t feedback_timeout_ms = 20;

};

template <>
struct MotorInstanceConfig<MotorKind::DM> {
    CanPort* port = nullptr;
    uint16_t master_id = 0;
    uint16_t can_id = 0;
    bool reverse = false;
    uint32_t feedback_timeout_ms = 20;

};

template <>
struct MotorInstanceConfig<MotorKind::LZ> {
    CanPort* port = nullptr;
    uint8_t motor_id = 0;
    uint8_t host_id = 0xff;
    LzControlMode mode = LzControlMode::Motion;
    bool reverse = false;
    uint32_t feedback_timeout_ms = 20;

};

} // namespace mr::motor
