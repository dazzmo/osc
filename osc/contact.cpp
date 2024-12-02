#include "osc/contact.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/osc.hpp"

namespace osc {

// Return linear constraint
bopt::linear_constraint<double>::shared_ptr
FrictionConeConstraint::to_constraint(const model_t &model) const {
  // https://scaron.info/robotics/friction-cones.html

  // Create friction constraint
  vector_sym_t f = create_symbolic_vector("f", 3);
  vector_sym_t n = create_symbolic_vector("n", 3);
  vector_sym_t t = create_symbolic_vector("t", 3);
  vector_sym_t b = create_symbolic_vector("b", 3);
  sym_t mu = sym_t::sym("mu");

  // Inner approximation of friction cone
  sym_t mu_tilde = mu / sqrt(2.0);
  sym_t limit = mu_tilde * f.dot(n);

  // Constraint to indicate | f.dot(t) | <= mu * f.dot(n)
  sym_t constraint = sym_t::zeros(4);
  constraint(0) = f.dot(t) - limit;
  constraint(1) = -f.dot(t) - limit;
  constraint(2) = f.dot(b) - limit;
  constraint(3) = -f.dot(b) - limit;

  return bopt::casadi::linear_constraint<double>::create(
      constraint, casadi::eigen_to_casadi(f),
      sym_vector_t({casadi::eigen_to_casadi(n), casadi::eigen_to_casadi(t),
                    casadi::eigen_to_casadi(b), mu}),
      bopt::bound_type::Negative);
}

// Return linear constraint
bopt::linear_constraint<double>::shared_ptr NoSlipConstraint::to_constraint(
    const model_t &model) const {
  // Compute the target frame in the contact frame of the model
  vector_sym_t q = create_symbolic_vector("q", model.nq);
  vector_sym_t v = create_symbolic_vector("v", model.nv);
  vector_sym_t a = create_symbolic_vector("a", model.nv);
  vector_sym_t e = create_symbolic_vector("e", contact_->dimension);

  // Compute frame state
  model_sym_t model_sym = model.cast<sym_t>();
  data_sym_t data_sym(model_sym);

  // Compute the kinematic tree state of the system
  pinocchio::forwardKinematics(model_sym, data_sym, q, v, a);
  pinocchio::updateFramePlacements(model_sym, data_sym);

  // Compute Jacobian and bias
  matrix_sym_t J(3, model.nv);
  vector_sym_t bias(3);

  contact_->jacobian(model_sym, data_sym, J);
  contact_->bias_acceleration(model_sym, data_sym, bias);

  vector_sym_t no_slip = J * a - bias - e;

  return bopt::casadi::linear_constraint<double>::create(
      casadi::eigen_to_casadi(no_slip), casadi::eigen_to_casadi(a),
      sym_vector_t({casadi::eigen_to_casadi(q), casadi::eigen_to_casadi(v),
                    casadi::eigen_to_casadi(e)}),
      bopt::bound_type::Equality);
}

}  // namespace osc