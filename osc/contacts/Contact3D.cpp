#include "osc/contacts/Contact3D.hpp"

namespace osc {

void Contact3D::computeError(const State &state, Eigen::Ref<Vector> e,
                             Eigen::Ref<Vector> dot_e) const {
    e = state.getTransformFrameToWorld(frame()).translation() - getTarget();
    dot_e = -state.getFrameVelocity(state.getFrameIndex(frame())).linear();
}

void Contact3D::computeJacobian(const State &state,
                                Eigen::Ref<Matrix> jacobian) const {
    jacobian = state.computeFrameJacobian(state.getFrameIndex(frame()))
                   .topRows<DIMENSION>();
}

void Contact3D::computeAccelerationBias(const State &state,
                                        Eigen::Ref<Vector> bias) const {
    bias = state.getFrameClassicalAcceleration(state.getFrameIndex(frame()))
               .linear();
}

void Contact3D::computeContactJacobian(const State &state,
                                       Eigen::Ref<Matrix> jacobian) const {
    jacobian = state
                   .computeFrameJacobian(state.getFrameIndex(frame()),
                                         pinocchio::WORLD)
                   .topRows<DIMENSION>();
}

}  // namespace osc