#pragma once

#include "osc/tasks/motion.hpp"

namespace osc {

/**
 * @brief Task for tracking joint positions and their velocities
 *
 */
class PostureTask : public MotionTask {
 public:
  PostureTask(const model_t &model);

  index_t dim() const override { return nj_; }

  void set_reference(const TrajectoryReference &ref) override {
    reference_ = ref;
  }

  void compute(const model_t &model, data_t &data, const vector_t &q,
               const vector_t &v) override;

  void compute_jacobian(const model_t &model, data_t &data,
                        const vector_t &q) override {
    // Joint jacobian is constant
  }

  void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                  const vector_t &q,
                                  const vector_t &v) override {
    // dJdq is constant and zero
  }

  void compute_error(const model_t &model, data_t &data, const vector_t &q,
                     const vector_t &v) override ;

 private:
  TrajectoryReference reference_;
  index_t nj_;
  std::vector<int> q_indices_;
  std::vector<int> v_indices_;
};

}  // namespace osc