#pragma once

#include "osc/tasks/motion.hpp"

namespace osc {

class CentreOfMassTask : public MotionTask {
 public:
  CentreOfMassTask(const model_t &model,
                   const std::string &reference_frame = "universe")
      : MotionTask(model, reference_frame) {
    // Error
    e_ = vector_t::Zero(dim());
    e_dot_ = vector_t::Zero(dim());

    // Default gains
    Kp_ = vector_t::Ones(dim());
    Kd_ = vector_t::Ones(dim());

    // Desired task acceleration
    xacc_des_ = vector_t::Zero(dim());
  }

  index_t dim() const override { return 3; }

  void compute(const model_t &model, data_t &data, const vector_t &q,
               const vector_t &v) override;

  void compute_jacobian(const model_t &model, data_t &data,
                        const vector_t &q) override;

  void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                  const vector_t &q,
                                  const vector_t &v) override;

  void compute_error(const model_t &model, data_t &data, const vector_t &q,
                     const vector_t &v) override;

 private:
};

}  // namespace osc