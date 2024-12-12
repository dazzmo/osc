
#pragma once

#include "osc/se3.hpp"
#include "osc/tasks/motion.hpp"

namespace osc {

/**
 * @brief Task for tracking a specified frame
 *
 */
class FrameTask : public MotionTask, public FrameSE3 {
 public:
  using Type = FrameSE3::Type;

  FrameTask(const model_t &model, const std::string &frame_name,
            const Type &type = Type::Full);

  static std::shared_ptr<FrameTask> create(const model_t &model,
                                           const std::string &frame_name,
                                           const Type &type = Type::Full);

  index_t dim() const override { return FrameSE3::dim(); }

  void set_reference(const TrajectoryReference &ref) override {
    reference_.set(ref);
  }
  void set_reference(const pinocchio::SE3 &ref) { reference_.position = ref; }
  void set_reference(const SE3TrajectoryReference &ref) { reference_ = ref; }

  const SE3TrajectoryReference &get_reference() const { return reference_; }

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
  SE3TrajectoryReference reference_;
};

}  // namespace osc