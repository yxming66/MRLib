#include <type_traits>

#include "device/joint/motor_actuator_adapter.hpp"

static_assert(std::is_base_of<mr::IActuator, mr::DmJ4310Actuator>::value,
              "the DM adapter must implement the generic actuator contract");
static_assert(std::is_base_of<mr::IActuator, mr::LzActuator>::value,
              "the LZ adapter must implement the generic actuator contract");
