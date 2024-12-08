#include "osc/dynamics.hpp"

#include <pinocchio/algorithm/crba.hpp>

namespace osc {

InverseDynamics::InverseDynamics(const index_t &nq, const index_t &nv,
                                 const index_t &nu) {
  M_ = matrix_t::Zero(nv, nv);
  h_ = vector_t::Zero(nv);
  B_ = matrix_t::Zero(nv, nu);

  N_ = matrix_t::Identity(nv, nv);
  gamma_ = vector_t::Zero(nv);
}

void InverseDynamics::compute(const model_t &model, data_t &data,
                              const vector_t &q, const vector_t &v) {
  compute_inertial_matrix(model, data, q, v);
  compute_nonlinear_effects(model, data, q, v);
  compute_actuation_map(model, data, q, v);
}

void InverseDynamics::compute_inertial_matrix(const model_t &model,
                                              data_t &data, const vector_t &q,
                                              const vector_t &v) {
  M_ = pinocchio::crba(model, data, q);
}

void InverseDynamics::compute_nonlinear_effects(const model_t &model,
                                                data_t &data, const vector_t &q,
                                                const vector_t &v) {
  // h_ = pinocchio::nonlinearEffects(model, data, q, v);
}

const matrix_t &InverseDynamics::get_inertial_matrix() const { return M_; }
const vector_t &InverseDynamics::get_nonlinear_terms() const { return h_; }
const matrix_t &InverseDynamics::get_actuation_map() const { return B_; }

const matrix_t &InverseDynamics::get_holonomic_constraint_jacobian() const {
  return jacobian_h_;
}

const vector_t &InverseDynamics::get_holonomic_constraint_jacobian_dot_q_dot()
    const {
  return jacobian_dot_q_dot_h_;
}

}  // namespace osc