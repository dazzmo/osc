#include "osc/tasks/CentreOfMass.hpp"
namespace osc {

void CentreOfMassTask::computeError(const State &state, Eigen::Ref<Vector> e,
                                    Eigen::Ref<Vector> dot_e) const {
    e = state.getCentreOfMass() - getTarget().position;
    dot_e = state.getCentreOfMass() - getTarget().velocity;
}

void CentreOfMassTask::computeJacobian(const State &state,
                                       Eigen::Ref<Matrix> jac) const {
    jac = state.computeCentreOfMassJacobian();
}

void CentreOfMassTask::computeAccelerationBias(const State &state,
                                               Eigen::Ref<Vector> bias) const {
                                                
                                               }

}  // namespace osc