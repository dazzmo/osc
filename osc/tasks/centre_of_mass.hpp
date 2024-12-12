#pragma once

#include "osc/tasks/motion.hpp"

namespace osc {

class CentreOfMassTask : public MotionTask {
 public:
  CentreOfMassTask(const model_t &model);

  index_t dim() const override { return 3; }

  void set_reference_frame(const model_t &model, const string_t &frame) {
    reference_frame_ = frame;
    reference_frame_id_ = model.getFrameId(reference_frame_);
    if (reference_frame_id_ == model.frames.size()) {
      assert("Model does not have specified reference frame");
    }
  }

  const string_t &reference_frame() const { return reference_frame_; }
  const index_t &reference_frame_id() const { return reference_frame_id_; }

  void set_reference(const TrajectoryReference &ref) override {
    reference_ = ref;
  }

  const TrajectoryReference &get_reference() const { return reference_; }

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
  string_t reference_frame_;
  index_t reference_frame_id_;

  TrajectoryReference reference_;
  matrix_t jacobian_full_;
};

}  // namespace osc