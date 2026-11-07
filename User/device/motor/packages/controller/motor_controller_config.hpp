#pragma once
#include "component/pid.h"
#include "component/timer.hpp"

namespace mr::motor {

struct MotorControllerConfig {
    // 速度环 PID。用于模拟速度、模拟位置的内环；nullptr 表示不可用。
    const KPID_Params_t* velocity_pid;
    // 位置环 PID。用于模拟位置或位置到力矩；nullptr 表示不可用。
    const KPID_Params_t* position_pid;
    // 控制器采样频率 Hz。<=0 时回退到 PID 库默认频率。
    float sample_freq;

    // 反馈位置和速度的一阶低通截止频率 Hz。<=0 时禁用反馈滤波。
    float feedback_lowpass_cutoff_hz;
    // 输出力矩的一阶低通截止频率 Hz。<=0 时禁用输出滤波。
    float output_lowpass_cutoff_hz;

    // 控制周期使用的单调微秒时钟。为空时不读取硬件时钟，使用 sample_freq
    // 推导的固定周期；板级工程应显式注入自己的 BSP 时钟。
    mr::comp::ClockUsFn clock_us;

        constexpr MotorControllerConfig()
                : velocity_pid(nullptr),
                    position_pid(nullptr),
                    sample_freq(0.0f),
                    feedback_lowpass_cutoff_hz(0.0f),
                    output_lowpass_cutoff_hz(0.0f),
                    clock_us(nullptr) {}

        constexpr MotorControllerConfig(const KPID_Params_t* velocity,
                                                                        const KPID_Params_t* position,
                                                                        float sample,
                                                                        float feedback_cutoff,
                                                                        float output_cutoff)
                : velocity_pid(velocity),
                    position_pid(position),
                    sample_freq(sample),
                    feedback_lowpass_cutoff_hz(feedback_cutoff),
                    output_lowpass_cutoff_hz(output_cutoff),
                    clock_us(nullptr) {}

        constexpr MotorControllerConfig(const KPID_Params_t* velocity,
                                         const KPID_Params_t* position,
                                         float sample,
                                         float feedback_cutoff,
                                         float output_cutoff,
                                         mr::comp::ClockUsFn clock)
                : velocity_pid(velocity),
                  position_pid(position),
                  sample_freq(sample),
                  feedback_lowpass_cutoff_hz(feedback_cutoff),
                  output_lowpass_cutoff_hz(output_cutoff),
                  clock_us(clock) {}
};

template <typename MotorType>
struct MotorControllerDefaults {
    static constexpr MotorControllerConfig Config() {
        return MotorControllerConfig(nullptr, nullptr, 0.0f, 0.0f, 0.0f);
    }
};

} // namespace mr::motor

