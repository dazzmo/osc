#include "osc/tasks/Frame.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {
FrameTask::FrameTask(const String &frame)
    : Task<FrameTarget>(DIMENSION), frame_(frame) {}

void FrameTask::computeError(const State &state, Eigen::Ref<Vector> e,
                             Eigen::Ref<Vector> dot_e) const {
  const auto oMf = state.getTransformFrameToWorld(frame_);
  const auto &oMt = getTarget().pose;

  // Compute the error
  const auto fMt = oMf.actInv(oMt);
  e = pinocchio::log6(fMt).toVector();

  std::cout << "oMf (actual) = " << oMf << std::endl;
  std::cout << "oMf (target) = " << oMt << std::endl;

  // Compute the rate of error
  dot_e = (getTarget().velocity -
           state.getFrameVelocity(state.getFrameIndex(frame())))
              .toVector();
}

void FrameTask::computeJacobian(const State &state,
                                Eigen::Ref<Matrix> jac) const {
  jac = state.computeFrameJacobian(state.getFrameIndex(frame()));
}

void FrameTask::computeAccelerationBias(const State &state,
                                        Eigen::Ref<Vector> bias) const {
  const auto a =
      state.getFrameClassicalAcceleration(state.getFrameIndex(frame()));
  bias = a.toVector();
}

}  // namespace osc