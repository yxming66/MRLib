// Compile only, including complete template bodies on the embedded toolchain.
#include "component/cmd_center/cmd_center.hpp"
#include "component/controller/cascade.hpp"
#include "component/controller/lqr.hpp"
#include "component/filter/ekf.hpp"
#include "component/filter/kf.hpp"
#include "component/trajectory/s_curve.hpp"
#include "robotics/arm/control/arm_controller.hpp"

template class mr::comp::filter::kf<6, 3, 2>;
template class mr::comp::filter::ekf<6, 3, 2>;
template class mr::comp::traj::SynchronizedProfile<6, mr::comp::traj::SCurveProfile>;
template class mr::comp::cntlr::lqr<6, 2, 4>;
template class mr::robotics::arm::control::ArmController<2>;
template class mr::robotics::arm::control::ArmController<3>;
template class mr::robotics::arm::control::ArmController<6>;

namespace {
struct Input {
  uint32_t timestamp_ms() const { return 0; }
};
struct Builder {
  bool operator()(const Input&, int& out) const {
    out = 1;
    return true;
  }
};
struct Safe {
  bool operator()(int& out) const {
    out = 0;
    return true;
  }
};
}  // namespace
void CompileCommandCenter() {
  cmd::Center<1, 1, 3> center;
  center.input<Input>("input");
  center.output<int>("out", [](int) { return true; })
      .safe<Safe>()
      .arbitrate(cmd::HighestPriority{cmd::TieBreak::Freshest})
      .routes(cmd::from<Input, Builder>());
  center.tick(1).publish();
}
