#pragma once

#include "osc/common.hpp"

namespace osc {

class HolonomicExpression {
 public:
  HolonomicExpression() : dimension(0) {}
  HolonomicExpression(const index_t &dimension) : dimension(dimension) {}

  /**
   * @brief Dimension of the holonomic expression.
   *
   */
  index_t dimension;

  virtual void update_parameters(const model_t &model, data_t &data) {}

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
  virtual void jacobian(const model_t &model, data_t &data, matrix_t &J) = 0;

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
  virtual void bias_acceleration(const model_t &model, data_t &data,
                                 vector_t &bias) = 0;

  /**
   * \copydoc osc::AbstractTask::jacobian()
   */
  virtual void jacobian(const model_sym_t &model, data_sym_t &data,
                        matrix_sym_t &J) {
    throw std::runtime_error("symbolic jacobian() not implemented");
  }

  /**
   * \copydoc osc::AbstractTask::bias_acceleration()
   */
  virtual void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                                 vector_sym_t &bias) {
    throw std::runtime_error("symbolic bias_acceleration() not implemented");
  }
};

class HolonomicConstraint : public HolonomicExpression {
 public:
  HolonomicConstraint(const index_t &dimension) : HolonomicExpression(dimension) {
    lambda = bopt::create_variable_vector("lambda", dimension);
  }

  // Associated variables for constraint forces
  std::vector<bopt::variable> lambda;
};

}  // namespace osc