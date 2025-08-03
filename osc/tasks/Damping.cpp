#include "osc/tasks/Damping.hpp"

namespace osc {

void DampingTask::computeError(const State &state, Eigen::Ref<Vector> e,
                               Eigen::Ref<Vector> dot_e) const {
    e.setZero();
    dot_e = state.v() - getTarget();
}

void DampingTask::computeJacobian(const State &state,
                                  Eigen::Ref<Matrix> jac) const {
    if (state.hasFloatingBase()) {
        jac.rightCols(state.nv() - 6).setIdentity();
    } else {
        jac.setIdentity();
    }
}

void DampingTask::computeAccelerationBias(const State &state,
                                          Eigen::Ref<Vector> bias) const {
    bias.setZero();
}

}  // namespace osc
