#pragma once

#include "osc/common.hpp"

namespace osc {

class InverseDynamics {
 public:
  InverseDynamics(const index_t &nq, const index_t &nv, const index_t &nu);

  /**
   * @brief Dimension of the holonomic constraints acting on the dynamics
   *
   * @return index_t
   */
  virtual index_t dim_constraints() = 0;

  void compute(const model_t &model, data_t &data, const vector_t &q,
               const vector_t &v);

  virtual void compute_inertial_matrix(const model_t &model, data_t &data,
                                       const vector_t &q, const vector_t &v);
  virtual void compute_nonlinear_effects(const model_t &model, data_t &data,
                                         const vector_t &q, const vector_t &v);
  virtual void compute_actuation_map(const model_t &model, data_t &data,
                                     const vector_t &q, const vector_t &v) = 0;

  virtual void compute_holonomic_constraint_projector_matrix(
      const model_t &model, data_t &data, const vector_t &q,
      const vector_t &v) {}

  virtual void compute_holonomic_constraint_bias_vector(const model_t &model,
                                                        data_t &data,
                                                        const vector_t &q,
                                                        const vector_t &v) {}

  const matrix_t &get_inertial_matrix() const;
  const vector_t &get_nonlinear_terms() const;
  const matrix_t &get_actuation_map() const;

  virtual void get_holonomic_constraint_jacobian() const;
  virtual void get_holonomic_constraint_jacobian_dot_q_dot() const;

 private:
  matrix_t M_;
  vector_t h_;
  matrix_t B_;

  // Projection matrix for any holonomic constraints
  matrix_t jacobian_h_;
  // Bias vector for any holonomic constraints
  vector_t jacobian_dot_q_dot_h_;
};

}  // namespace osc