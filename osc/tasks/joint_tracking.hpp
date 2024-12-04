#pragma once

#include "osc/task/task.hpp"

namespace osc {

/**
 * @brief Task for tracking joint positions and their velocities
 * 
 */
class JointTrackingTask : public AbstractTask {
 public:
  JointTrackingTask(const model_t &model) : AbstractTask() {}

  struct reference {
    vector_t position;
    vector_t velocity;
  };

  void jacobian(const model_t &model, data_t &data, const vector_t &q,
                matrix_t &J) const override {
    // For all joints in the system, set the entries to 1
  }

  void bias_acceleration(const model_t &model, data_t &data, const vector_t &q,
                         const vector_t &v, vector_t &bias) const override {
    bias.setZero();
  }

  void jacobian(const model_sym_t &model, data_sym_t &data,
                const vector_sym_t &q, matrix_sym_t &J) const override {}

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         const vector_sym_t &q, const vector_sym_t &v,
                         vector_sym_t &bias) const override {
    bias.setZero();
  }

  reference target;
  string_t reference_frame;

  void evaluate_error(const model_t &model, const data_t &data, vector_t &e,
                      vector_t &e_dot) {
    // Compute centre of mass with respect to reference frame
    e.setZero();
    e_dot.setZero();
  }

 private:
};

}  // namespace osc