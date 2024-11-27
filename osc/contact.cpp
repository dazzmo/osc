#include "osc/contact.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/osc.hpp"

namespace osc {

ContactPoint::ContactPoint(const index_type &dim, const index_type &model_nq,
                           const index_type &model_nv, const string_t &target)
    : HolonomicConstraint(dim), frame(target) {
  parameters_v.epsilon = bopt::create_variable_vector("eps", dimension());
  parameters_d.epsilon = vector_t::Zero(dimension());

  parameters_v.n = bopt::create_variable_vector("n", dimension());
  parameters_d.n = vector_t::Zero(dimension());

  parameters_v.t = bopt::create_variable_vector("t", dimension());
  parameters_d.t = vector_t::Zero(dimension());

  parameters_v.b = bopt::create_variable_vector("b", dimension());
  parameters_d.b = vector_t::Zero(dimension());

  parameters_v.lambda_ub =
      bopt::create_variable_vector("lambda_ub", dimension());
  parameters_d.lambda_ub = vector_t::Zero(dimension());

  parameters_v.lambda_lb =
      bopt::create_variable_vector("lambda_lb", dimension());
  parameters_d.lambda_lb = vector_t::Zero(dimension());
}

matrix_sym_t ContactPoint::constraint_jacobian(const model_sym_t &model,
                                               const vector_sym_t &q) const {
  pinocchio::DataTpl<sym_t> data(model);

  // Compute the kinematic tree state of the system
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_sym_t J = matrix_sym_t::Zero(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId(frame),
                                  pinocchio::ReferenceFrame::WORLD, J);

  return J;
}

bopt::linear_constraint<ContactPoint3D::value_type>::shared_ptr
ContactPoint3D::create_friction_constraint(const model_sym_t &model) const {
  // https://scaron.info/robotics/friction-cones.html

  // Create friction constraint
  vector_sym_t f = create_symbolic_vector("f", dimension());
  vector_sym_t n = create_symbolic_vector("n", dimension());
  vector_sym_t t = create_symbolic_vector("t", dimension());
  vector_sym_t b = create_symbolic_vector("b", dimension());
  sym_t mu = sym_t::sym("mu");

  // Inner approximation of friction cone
  sym_t mu_tilde = mu / sqrt(2.0);

  sym_t limit = mu * f.dot(n);

  // Constraint to indicate | f.dot(t) | <= mu * f.dot(n)
  sym_t constraint = sym_t::zeros(4);
  constraint(0) = f.dot(t) - limit;
  constraint(1) = -f.dot(t) - limit;
  constraint(2) = f.dot(b) - limit;
  constraint(3) = -f.dot(b) - limit;

  sym_t f_s = casadi::eigen_to_casadi(f);
  sym_t n_s = casadi::eigen_to_casadi(n);
  sym_t t_s = casadi::eigen_to_casadi(t);
  sym_t b_s = casadi::eigen_to_casadi(b);

  return bopt::casadi::linear_constraint<value_type>::create(
      constraint, f_s, sym_vector_t({n_s, t_s, b_s, mu}),
      bopt::bound_type::Negative);
}

bopt::bounding_box_constraint<ContactPoint3D::value_type>::shared_ptr
ContactPoint3D::create_friction_bound_constraint(
    const model_sym_t &model) const {
  // Create bounding box constraint
  vector_sym_t fl = create_symbolic_vector("fl", dimension());
  vector_sym_t fu = create_symbolic_vector("fu", dimension());

  sym_t fl_s = casadi::eigen_to_casadi(fl);
  sym_t fu_s = casadi::eigen_to_casadi(fu);

  return bopt::casadi::bounding_box_constraint<value_type>::create(
      fl_s, fu_s, sym_vector_t({fl_s, fu_s}));
}

bopt::linear_constraint<ContactPoint3D::value_type>::shared_ptr
ContactPoint3D::create_no_slip_constraint(const model_sym_t &model) const {
  // Compute the target frame in the contact frame of the model
  vector_sym_t q = create_symbolic_vector("q", model.nq);
  vector_sym_t v = create_symbolic_vector("v", model.nv);
  vector_sym_t a = create_symbolic_vector("a", model.nv);
  vector_sym_t e = create_symbolic_vector("e", dimension());

  // Compute frame state
  pinocchio::DataTpl<sym_t> data(model);

  // Compute the kinematic tree state of the system
  pinocchio::forwardKinematics(model, data, q, v, a);
  pinocchio::updateFramePlacements(model, data);

  // Compute the acceleration of the target frame in its LOCAL frame
  pinocchio::MotionTpl<sym_t> acc = pinocchio::getFrameClassicalAcceleration(
      model, data, model.getFrameId(frame), pinocchio::WORLD);

  // Create expression evaluators
  sym_t q_s = casadi::eigen_to_casadi(q);
  sym_t v_s = casadi::eigen_to_casadi(v);
  sym_t a_s = casadi::eigen_to_casadi(a);
  sym_t e_s = casadi::eigen_to_casadi(e);

  // No slip condition => xacc = J qacc + \dot J qvel = epsilon
  sym_t constraint = casadi::eigen_to_casadi(acc.linear() - e);

  // todo - need to consider the frame transformation from contact frame to
  // todo - end-effector frame

  return bopt::casadi::linear_constraint<value_type>::create(
      constraint, a_s, sym_vector_t({q_s, v_s, e_s}));
}

void ContactPoint3D::add_to_program(OSC &osc_program) const {
  const model_sym_t &model = OSCComponent::get_model(osc_program);
  bopt::mathematical_program<double> &program =
      OSCComponent::get_program(osc_program);
  osc_variables &variables = OSCComponent::get_variables(osc_program);
  osc_parameters &parameters = OSCComponent::get_parameters(osc_program);

  // Parameters
  program.add_parameters(parameters_v.epsilon);
  program.add_parameters(parameters_v.n);
  program.add_parameters(parameters_v.t);
  program.add_parameters(parameters_v.b);
  program.add_parameters(parameters_v.lambda_ub);
  program.add_parameters(parameters_v.lambda_lb);
  program.add_parameter(parameters_v.mu);

  // Create constraints
  auto friction_cone = create_friction_constraint(model);
  auto friction_bound = create_friction_bound_constraint(model);
  auto no_slip = create_no_slip_constraint(model);

  // Create new variables
  std::vector<bopt::variable> lambda =
      bopt::create_variable_vector("lambda", dimension());
  // Add these to the program
  variables.lambda.insert(variables.lambda.end(), lambda.begin(), lambda.end());

  // Bind to program
  program.add_linear_constraint(
      friction_cone,
      // Variables
      {lambda},
      // contact-specific parameters
      {parameters_v.n, parameters_v.t, parameters_v.b, {parameters_v.mu}});

  program.add_linear_constraint(no_slip,
                                // Variables
                                {variables.a},
                                // Parameters
                                {parameters.q, parameters.v,
                                 // Task-specific parameters
                                 parameters_v.epsilon});

  program.add_bounding_box_constraint(
      friction_bound,
      // Variables
      {lambda},
      // Parameters
      {parameters_v.lambda_lb, parameters_v.lambda_ub});
}

}  // namespace osc