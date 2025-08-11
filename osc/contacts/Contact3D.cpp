#include "osc/contacts/Contact3D.hpp"

namespace osc {

void Contact3D::computeError(const State &state, Eigen::Ref<Vector> e,
                             Eigen::Ref<Vector> dot_e) const {
  const auto &oMf = state.getTransformFrameToWorld(frame());
  const auto &fMc = contactTransform();
  const auto &fvo = state.getFrameVelocity(state.getFrameIndex(frame()));
  
  std::cout << "x (actual) = " << oMf.act(fMc).translation() << std::endl;
  std::cout << "x (target) = " << getTarget() << std::endl;
  
  e = getTarget() - (oMf.act(fMc)).translation();
  // Get body velocity of the contact frame in the contact frame
  dot_e = -fMc.actInv(fvo).linear();
}

void Contact3D::computeJacobian(const State &state,
                                Eigen::Ref<Matrix> jacobian) const {
  const auto &fMc = contactTransform();
  jacobian =
      (fMc.inverse().toActionMatrix() * state.computeFrameJacobian(state.getFrameIndex(frame())))
          .topRows<DIMENSION>();
}

void Contact3D::computeAccelerationBias(const State &state,
                                        Eigen::Ref<Vector> bias) const {
  const auto &fMc = contactTransform();
  const auto a =
      state.getFrameClassicalAcceleration(state.getFrameIndex(frame()));
  bias = (fMc.actInv(a)).linear();
}

void Contact3D::setTargetFromState(const State &state) {
  const auto &fMc = contactTransform();
  setTarget(state.getTransformFrameToWorld(frame()).act(fMc).translation());
}

}  // namespace osc