#pragma once

/*
 * 电机静态工厂：使用 placement new 在固定容量的静态存储区构造 MotorT。
 *
 * 快速用法：
 *   MotorInstanceConfig<MotorKind::RM> config{};
 *   config.can = BSP_CAN_1; config.id = 0x201;  // RM feedback CAN ID
 *   auto* motor = MotorFactory::Create<MotorKind::RM, MotorModel::M3508>(config);
 *   motor->Register();
 *
 * 控制周期：Update() -> SetTorque/SetPosition/SetMIT() -> CommitCommand()。
 * Create 返回 nullptr 表示该 MotorKind 的固定容量已用尽。
 */

// 电机静态工厂。使用方式：MotorFactory::Create<MotorKind, MotorModel>(config)
// 返回固定存储区中的 MotorT 指针；随后调用 Register、周期 Update 和控制接口。

// 强类型电机对象静态工厂。
// 负责在固定静态存储区内构造不同 kind/model 的 MotorT 实例，避免动态堆分配。

#include <new>
#include <stdint.h>

#include "device/motor/core/motor_install_spec.hpp"
#include "device/motor/core/motor_t.hpp"

namespace mr::motor {

inline constexpr uint8_t kRmMotorFactoryCapacity = 16;
inline constexpr uint8_t kDmMotorFactoryCapacity = 8;
inline constexpr uint8_t kLzMotorFactoryCapacity = 8;

class MotorFactory {
public:
    template <MotorKind Kind, MotorModel Model>
    static MotorT<Kind, Model>* Create(
        const MotorInstanceConfig<Kind>&  config,
        const MotorInstallSpec& install = kDirectDriveInstall,
        const MotorTemperatureProtectionConfig& temperature_protection = {}) {
        static_assert(MotorModelValidV<Kind, Model>, "Invalid motor kind/model combination");
        using MotorType = MotorT<Kind, Model>;
        static DriverSlot<MotorType> drivers[Capacity<Kind>()];
        static uint8_t count = 0U;
        if (count >= Capacity<Kind>()) {
            return nullptr;
        }
        MotorType* driver =
            new (&drivers[count].value) MotorType(config, install, temperature_protection);
        ++count;
        return driver;
    }

private:
    template <typename T>
    union DriverSlot {
        DriverSlot() {}
        ~DriverSlot() {}
        T value;
    };

    template <MotorKind Kind>
    static constexpr uint8_t Capacity();

};

template <>
inline constexpr uint8_t MotorFactory::Capacity<MotorKind::RM>() { return kRmMotorFactoryCapacity; }

template <>
inline constexpr uint8_t MotorFactory::Capacity<MotorKind::DM>() { return kDmMotorFactoryCapacity; }

template <>
inline constexpr uint8_t MotorFactory::Capacity<MotorKind::LZ>() { return kLzMotorFactoryCapacity; }

} // namespace mr::motor
