#include "osc/tasks/CentreOfMass.hpp"
namespace osc {

CentreOfMassTask::CentreOfMassTask() : Task<CentreOfMassTarget>(DIMENSION) {}

void CentreOfMassTask::computeError(const State &state, Eigen::Ref<Vector> e,
                                    Eigen::Ref<Vector> dot_e) const {
    e = getTarget().position - state.getCentreOfMass();
    std::cout << "c (actual) = " << state.getCentreOfMass() << std::endl;
    std::cout << "c (target) = " << getTarget().position << std::endl;
    dot_e = getTarget().velocity - state.getCentreOfMassVelocity();
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