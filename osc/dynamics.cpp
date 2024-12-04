#include "osc/dynamics.hpp"

namespace osc {

std::vector<bopt::variable>
AbstractSystemDynamics::get_constraint_force_vector() {
  std::vector<bopt::variable> lambda = {};
  for (const auto &constraint : constraints_) {
    lambda.insert(lambda.end(), constraint->lambda.begin(),
                  constraint->lambda.end());
  }
  return lambda;
}

// Return linear constraint
bopt::linear_constraint<double>::shared_ptr
SystemDynamicsConstraint::to_constraint(const model_t &model) const {
  // Compute the target frame in the contact frame of the model
  vector_sym_t q = create_symbolic_vector("q", model.nq);
  vector_sym_t v = create_symbolic_vector("v", model.nv);
  vector_sym_t a = create_symbolic_vector("a", model.nv);
  vector_sym_t u = create_symbolic_vector("u", dynamics_->nu());

  // Compute frame state
  model_sym_t model_sym = model.cast<sym_t>();
  data_sym_t data_sym(model_sym);

  // Compute the kinematic tree state of the system
  pinocchio::forwardKinematics(model_sym, data_sym, q, v, a);
  pinocchio::updateFramePlacements(model_sym, data_sym);

  // Create unconstrained dynamics
  vector_sym_t f;
  f = this->evaluate(model_sym, data_sym, q, v, a, u);

  // Add additional dynamics
  for (const auto &dyn : dynamics_) {
    f += dyn->evaluate(model_sym, data_sym, q, v, a, u);
  }

  // Add constraint forces
  vector_sym_t l;
  for (const auto &constraint : constraints_) {
    vector_sym_t lambda_i = create_symbolic_vector("l", constraint->dimension);
    matrix_sym_t J_i(constraint->dimension, model.nv);
    constraint->jacobian(model_sym, data_sym, q, J_i);
    f -= J_i.transpose() * lambda_i;
    // Add to force variables
    l.conservativeResize(l.size() + lambda_i.size());
    l.bottomRows(lambda_i.size()) = lambda_i;
  }

  // Create vector for linearisation
  vector_sym_t x(model.nv + nu_ + l.size());
  x << a, u, l;

  return bopt::casadi::linear_constraint<double>::create(
      casadi::eigen_to_casadi(f), casadi::eigen_to_casadi(x),
      sym_vector_t({casadi::eigen_to_casadi(q), casadi::eigen_to_casadi(v)}),
      bopt::bound_type::Equality);
}

}  // namespace osc