#include "osc/contact/contact.hpp"

namespace osc {

void FrictionContact3D::compute(const model_t &model, data_t &data,
                                const vector_t &q, const vector_t &v) {
  // https://scaron.info/robotics/friction-cones.html

  auto proj = [](const vector_t &u, const vector_t &v) {
    return (v.dot(u) / u.dot(u)) * u;
  };
  // Create basis from surface normal
  vector3_t t = vector3_t::UnitX(), b = vector3_t::UnitY();
  t = t - proj(n, t);
  b = b - proj(n, b) - proj(t, b);

  // Inner approximation of friction cone
  double mu_tilde = mu_ / sqrt(2.0);

  matrix_t &A = friction_cone_constraint_->A();
  matrix_t &b = friction_cone_constraint_->b();

  vector_t &lb = friction_cone_constraint_->lower_bound();
  vector_t &ub = friction_cone_constraint_->upper_bound();
  // Constraint to indicate | f.dot(t) | <= mu * f.dot(n)
  A.row(0) = t - mu_tilde * n_.transpose();
  A.row(1) = -t - mu_tilde * n_.transpose();
  // Constraint to indicate | f.dot(b) | <= mu * f.dot(n)
  A.row(2) = b - mu_tilde * n_.transpose();
  A.row(3) = -b - mu_tilde * n_.transpose();

  b.setZero();

  lb.setConstant(-std::numeric_limits<double>::max());
  ub.setZero();

  // Compute other quantities
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
}

void FrictionContact3D::compute_jacobian(const model_t &model, data_t &data,
                                         const vector_t &q) {
  pinocchio::getFrameJacobian(model, data, model.getFrameId(frame),
                              pinocchio::WORLD, jacobian_full_);

  // todo - investigate the effects of the chosen frame
  jacobian_ = jacobian_full_.topRows(3);
}

void FrictionContact3D::compute_jacobian_dot_q_dot(const model_t &model,
                                                   data_t &data,
                                                   const vector_t &q,
                                                   const vector_t &v) {
  pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
      model, data, model.getFrameId(frame), pinocchio::WORLD);

  jacobian_dot_q_dot_ = acc.linear();
}

}  // namespace osc