#include "osc/dynamics.hpp"

#include <pinocchio/algorithm/rnea.hpp>

#include "osc/osc.hpp"

namespace osc {

SystemDynamics::SystemDynamics(const model_t &model, const index_t &nu)
    : model(model.cast<sym_t>()) {
  // Compute the inverse dynamics of the system
  vector_sym_t tau;

  pinocchio::DataTpl<sym_t> data(this->model);

  // Create symbolic vectors for computation
  q = create_symbolic_vector("q", model.nq);
  v = create_symbolic_vector("v", model.nv);
  a = create_symbolic_vector("a", model.nv);
  u = create_symbolic_vector("u", nu);
  f.resize(0);

  // Start with M \ddot q + C(q, \dot q) \dot q + G(q)
  dynamics_ = pinocchio::rnea(this->model, data, q, v, a);
}

void SystemDynamics::add_additional_dynamics(AdditionalDynamics &dynamics) {
  dynamics.add_to_dynamics(*this);
}

void SystemDynamics::add_constraint(const HolonomicConstraint &constraint) {
  // // Add to dynamics
  // auto J = constraint.constraint_jacobian(model, q);

  // // Create constraint forces
  // vector_sym_t lambda = create_symbolic_vector("f", constraint.dimension());
  // dynamics_ -= J.transpose() * lambda;

  // // Add to running total of constraint force vector
  // f.conservativeResize(f.size() + lambda.size());
  // f.bottomRows(lambda.size()) << lambda;
}

void SystemDynamics::add_to_program(OSC &osc_program) const {
  // const model_sym_t &model = OSCComponent::get_model(osc_program);
  // bopt::mathematical_program<double> &program =
  //     OSCComponent::get_program(osc_program);
  // const osc_variables &variables = OSCComponent::get_variables(osc_program);
  // osc_parameters &parameters = OSCComponent::get_parameters(osc_program);

  // // todo - need to consider the frame transformation from contact frame to
  // // todo - end-effector frame

  // // Create vector for linearisation of dynamics
  // sym_t x = sym_t::vertcat(
  //     sym_vector_t({casadi::eigen_to_casadi(a), casadi::eigen_to_casadi(u),
  //                   casadi::eigen_to_casadi(f)}));

  // // Create variable vector
  // std::vector<bopt::variable> xv(variables.a.size() + variables.u.size() +
  //                                variables.lambda.size());
  // xv.insert(xv.end(), variables.a.begin(), variables.a.end());
  // xv.insert(xv.end(), variables.u.begin(), variables.u.end());
  // xv.insert(xv.end(), variables.lambda.begin(), variables.lambda.end());

  // // Create constraint and bind program variables
  // program.add_linear_constraint(
  //     bopt::casadi::linear_constraint<value_type>::create(
  //         casadi::eigen_to_casadi(dynamics_), x,
  //         sym_vector_t(
  //             {casadi::eigen_to_casadi(q), casadi::eigen_to_casadi(v)})),
  //     // Variables
  //     {xv},
  //     // Parameters
  //     {parameters.q, parameters.v});
}

}  // namespace osc