#include "osc/contacts/contact_3d.hpp"

namespace osc {

void FrictionContact3D::compute_jacobian(const model_t &model, data_t &data,
                                         const vector_t &q) {
  FrameSE3::compute_jacobian(model, data, jacobian_);
}

void FrictionContact3D::compute_jacobian_dot_q_dot(const model_t &model,
                                                   data_t &data,
                                                   const vector_t &q,
                                                   const vector_t &v) {
  FrameSE3::compute_jacobian_dot_q_dot(model, data, jacobian_dot_q_dot_);
}

void FrictionContact3D::compute_error(const model_t &model, data_t &data,
                                      const vector_t &q, const vector_t &v) {
  FrameSE3::compute_error(model, data, get_reference().position,
                          get_reference().velocity, e_, e_dot_);
}

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

  VLOG(10) << "n = " << get_surface_normal().transpose();
  VLOG(10) << "t = " << surface_tangent.transpose();
  VLOG(10) << "b = " << surface_binormal.transpose();

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
  // f_min < f.dot(n) < f_max
  A.row(4) << get_surface_normal().transpose();
  A.row(5) << -get_surface_normal().transpose();

  b.setZero();
  b(4) = -get_max_normal_force();
  b(5) = get_min_normal_force();

  lb.setConstant(-std::numeric_limits<double>::max());
  ub.setZero();

  // Compute jacobian and derivative
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
  compute_error(model, data, q, v);

  // Compute desired task acceleration
  xacc_des_ =
      Kp_.asDiagonal() * get_error() + Kd_.asDiagonal() * get_error_dot();

  xacc_des_ += get_reference().acceleration.linear();
}

}  // namespace osc