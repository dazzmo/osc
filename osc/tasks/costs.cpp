#include "osc/tasks/costs.hpp"

namespace osc {

WeightedTaskCost::WeightedTaskCost(const model_t &model,
                                   const std::shared_ptr<AbstractTask> &task)
    : AbstractTaskCost(model, task), pid(task->dimension) {
  parameters.w = bopt::create_variable_vector("w", task->dimension);
  parameters.x = bopt::create_variable_vector("x", task->dimension);
  w = vector_t::Ones(task->dimension);
  x = vector_t::Zero(task->dimension);

  pid.resize(task->dimension);
  pid.p.setOnes();
  pid.i.setZero();
  pid.d.setOnes();
}

bopt::quadratic_cost<double>::shared_ptr WeightedTaskCost::to_cost(
    const model_t &model) const {
  // Compute the target frame in the contact frame of the model
  vector_sym_t q = create_symbolic_vector("q", model.nq);
  vector_sym_t v = create_symbolic_vector("v", model.nv);
  vector_sym_t a = create_symbolic_vector("a", model.nv);
  vector_sym_t e = create_symbolic_vector("e", task_->dimension);

  // Compute frame state
  model_sym_t model_sym = model.cast<sym_t>();
  data_sym_t data_sym(model_sym);

  // Compute the kinematic tree state of the system
  pinocchio::forwardKinematics(model_sym, data_sym, q, v, a);
  pinocchio::updateFramePlacements(model_sym, data_sym);

  // Compute Jacobian and bias
  matrix_sym_t J(task_->dimension, model.nv);
  vector_sym_t bias(task_->dimension);

  task_->jacobian(model_sym, data_sym, q, J);
  task_->bias_acceleration(model_sym, data_sym, q, v, bias);

  // Set difference between task second derivative and desired
  vector_sym_t error = J * a - bias - e;

  vector_sym_t w = create_symbolic_vector("w", task_->dimension);

  sym_t cost = error.transpose() * w.asDiagonal() * error;

  return bopt::casadi::quadratic_cost<double>::create(
      cost, casadi::eigen_to_casadi(a),
      sym_vector_t({casadi::eigen_to_casadi(q), casadi::eigen_to_casadi(v),
                    casadi::eigen_to_casadi(w), casadi::eigen_to_casadi(e)}));
}
}  // namespace osc