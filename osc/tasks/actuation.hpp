#pragma once

#include "osc/tasks/base.hpp"

namespace osc {

/**
 * @brief Tasks associated with motion tracking, and PD-based error correction
 *
 */
class ActuationTask : public TaskBase {
 public:
  ActuationTask(const model_t &model) {}

  /**
   * @brief Dimension of the task expression
   *
   * @return index_t
   */
  virtual index_t dim() = 0;

  const matrix_t &jacobian() const { return jacobian_; }

  virtual void compute_jacobian(const model_t &model, data_t &data,
                                const vector_t &q, const vector_t &v) = 0;

 protected:
  matrix_t jacobian_;

 private:
};

/**
 * @brief Task designed to minimise the value of ||u||^2
 *
 */
class MinimiseActuationTask : public ActuationTask {
 public:
  MinimiseActuationTask(const model_t &model, const index_t &nu)
      : ActuationTask(model) {
    jacobian_ = matrix_t::Identity(nu_, nu_);
  }

  index_t dim() override { return nu_; }

  void compute(const model_t &model, data_t &data, const vector_t &q,
               const vector_t &v) override {
    compute_jacobian(model, data, q, v);
  }

  void compute_jacobian(const model_t &model, data_t &data, const vector_t &q,
                        const vector_t &v) override {
    jacobian_.setIdentity();
  }

 protected:
 private:
  index_t nu_;
};

class ActuationBounds : public BoundConstraint {
 public:
  ActuationBounds(const index_t &nu) : BoundConstraint(nu) {}

 private:
};

}  // namespace osc