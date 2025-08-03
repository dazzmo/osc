#include "osc/tasks/Posture.hpp"

namespace osc {

PostureTask::PostureTask(const State &state) : Task<Vector>(state.nv()) {}

void PostureTask::computeError(const State &state, Eigen::Ref<Vector> e,
                               Eigen::Ref<Vector> dot_e) const {
    e = pinocchio::difference(state.model(), getTarget(), state.q());
    dot_e.setZero();
}

void PostureTask::computeJacobian(const State &state,
                                  Eigen::Ref<Matrix> jac) const {
    if (state.hasFloatingBase()) {
        jac.rightCols(state.nv() - 6).setIdentity();
    } else {
        jac.setIdentity();
    }
}

void PostureTask::computeAccelerationBias(const State &state,
                                          Eigen::Ref<Vector> bias) const {
    bias.setZero();
}

}  // namespace osc
