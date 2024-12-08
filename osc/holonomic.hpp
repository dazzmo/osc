#pragma once

#include "osc/common.hpp"

namespace osc {

/**
 * @brief Expression of the form x = f(q), which can be differentiated to
 * achieve the forms $\f \dot x = J(q) \dot q \f$ and \f$ \ddot x = J(q) \ddot q
 * + \dot J(q) \dot q \f$
 *
 */
class HolonomicConstraint {
 public:
  HolonomicConstraint() {}

  /**
   * @brief Dimension of the task expression
   *
   * @return index_t
   */
  virtual index_t dim() const = 0;

  const matrix_t &jacobian() const { return jacobian_; }
  const vector_t &jacobian_dot_q_dot() const { return jacobian_dot_q_dot_; }

  /**
   * @brief Numerical evaluation of a task Jacobian
   *
   * @param model
   * @param data
   * @param J The Jacobian of the task (i.e. \grad J \grad q)
   *
   * @note The method assumes that model and data are evaluated to a given
   * state, it does not perform any forward kinematics or dynamics.
   */
  virtual void compute_jacobian(const model_t &model, data_t &data,
                                const vector_t &q) = 0;

  /**
   * @brief Numerical evaluation of a task acceleration bias of the form \dot J
   * \dot q
   *
   * @param model
   * @param data
   * @param bias The product \dot J \dot q, referred to as the acceleration bias
   * of the task.
   *
   * @note The method assumes that model and data are evaluated to a given
   * state, it does not perform any forward kinematics or dynamics.
   */
  virtual void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                          const vector_t &q,
                                          const vector_t &q_dot) = 0;

 protected:
  matrix_t jacobian_;
  vector_t jacobian_dot_q_dot_;
};

}  // namespace osc