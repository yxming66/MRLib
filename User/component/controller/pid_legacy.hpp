#pragma once
/* Optional adapter for existing KPID tables. New code uses pid_config. */
#include "component/controller/pid.hpp"
#include "component/pid.h"

namespace mr::comp::cntlr {
inline bool TryLegacyPidConfig(const KPID_Params_t* params, KPID_Mode_t mode,
                                float sample_hz, pid_config* out) {
  if (out == nullptr) return false;
  MrPidConfig config;
  const bool valid = PID_ConfigFromLegacy(&config, mode, sample_hz, params);
  *out = pid::FromCoreConfig(config);
  return valid;
}
}  // namespace mr::comp::cntlr
