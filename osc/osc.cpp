#include "osc/osc.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

void OSC::loop(const vector_t &q, const vector_t &v) {
  assert(is_initialised_ && "OSC is not initialised!");
  // Update references for all tasks
  for (std::size_t i = 0; i < q.size(); ++i) {
    program_.set_parameter(parameters_.q[i], q[i]);
  }
  for (std::size_t i = 0; i < v.size(); ++i) {
    program_.set_parameter(parameters_.v[i], v[i]);
  }

  data_t data(model);

  // Update pinocchio model for computations
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::centerOfMass(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  // Update task errors
  for (auto &cost : task_costs_) {
    pid_error<double> e(cost->task()->dimension);
    cost->task()->evaluate_error(model, data, e.error, e.error_dot);
    // Update desired task acceleration
    vector_t a = pid<double>::compute_error(cost->pid, e);
    // Update desired task error
    for (std::size_t i = 0; i < a.size(); ++i) {
      program_.set_parameter(cost->parameters.x[i], a[i]);
    }
  }

  // Once updated, solve the program
  qp_->solve();

  // if(reset) qp.reset()
}

}  // namespace osc