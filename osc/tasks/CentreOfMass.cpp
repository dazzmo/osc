#include "osc/tasks/CentreOfMass.hpp"
namespace osc {

CentreOfMassTask::CentreOfMassTask() : Task<CentreOfMassTarget>(DIMENSION) {}

void CentreOfMassTask::computeError(const State &state, Eigen::Ref<Vector> e,
                                    Eigen::Ref<Vector> dot_e) const {
    e = state.getCentreOfMass() - getTarget().position;
    dot_e = state.getCentreOfMassVelocity() - getTarget().velocity;
}

void CentreOfMassTask::computeJacobian(const State &state,
                                       Eigen::Ref<Matrix> jac) const {
    jac = state.computeCentreOfMassJacobian();
}

void CentreOfMassTask::computeAccelerationBias(const State &state,
                                               Eigen::Ref<Vector> bias) const {
    bias = state.getCentreOfMassAcceleration();
}

}  // namespace osc