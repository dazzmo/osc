#include "osc/contacts/contact.hpp"

namespace osc {

void FrictionContact3D::compute(const model_t &model, data_t &data,
                                const vector_t &q, const vector_t &v) {
  // https://scaron.info/robotics/friction-cones.html

  auto proj = [](const vector_t &u, const vector_t &v) {
    return (v.dot(u) / u.dot(u)) * u;
  };
  // Create basis from surface normal
  vector3_t surface_tangent = vector3_t::UnitX(),
            surface_binormal = vector3_t::UnitY();
  surface_tangent =
      surface_tangent - proj(get_surface_normal(), surface_tangent);
  surface_binormal = surface_binormal -
                     proj(get_surface_normal(), surface_binormal) -
                     proj(surface_tangent, surface_binormal);

  // Inner approximation of friction cone
  double mu_tilde = mu_ / sqrt(2.0);

  matrix_t &A = friction_cone_constraint_->A();
  matrix_t &b = friction_cone_constraint_->b();

  vector_t &lb = friction_cone_constraint_->lower_bound();
  vector_t &ub = friction_cone_constraint_->upper_bound();
  // Constraint to indicate | f.dot(t) | <= mu * f.dot(n)
  A.row(0) << surface_tangent.transpose() -
                  mu_tilde * get_surface_normal().transpose();
  A.row(1) << -surface_tangent.transpose() -
                  mu_tilde * get_surface_normal().transpose();
  // Constraint to indicate | f.dot(b) | <= mu * f.dot(n)
  A.row(2) << surface_binormal.transpose() -
                  mu_tilde * get_surface_normal().transpose();
  A.row(3) << -surface_binormal.transpose() -
                  mu_tilde * get_surface_normal().transpose();

  b.setZero();

  lb.setConstant(-std::numeric_limits<double>::max());
  ub.setZero();

  // Compute other quantities
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
}

void FrictionContact3D::compute_jacobian(const model_t &model, data_t &data,
                                         const vector_t &q) {
  pinocchio::getFrameJacobian(model, data, frame_id(), pinocchio::WORLD,
                              jacobian_full_);

  // todo - investigate the effects of the chosen frame
  jacobian_ = jacobian_full_.topRows(3);
}

void FrictionContact3D::compute_jacobian_dot_q_dot(const model_t &model,
                                                   data_t &data,
                                                   const vector_t &q,
                                                   const vector_t &v) {
  pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
      model, data, frame_id(), pinocchio::WORLD);

  jacobian_dot_q_dot_ = acc.linear();
}

}  // namespace osc