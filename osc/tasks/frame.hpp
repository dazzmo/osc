
#pragma once

#include "osc/tasks/motion.hpp"

namespace osc {

/**
 * @brief Task for tracking a specified frame
 *
 */
class FrameTask : public MotionTask {
 public:
  enum class Type { Position = 0, Orientation, Full };

  FrameTask(const model_t &model, const std::string &frame_name,
            const Type &type = Type::Full,
            const std::string &reference_frame = "universe");

  static std::shared_ptr<FrameTask> create(
      const model_t &model, const std::string &frame_name,
      const Type &type = Type::Full,
      const std::string &reference_frame = "universe");

  index_t dim() const override {
    if (type == Type::Position || type == Type::Orientation) {
      return 3;
    } else {
      return 6;
    }
  }

  const string_t &frame() const { return frame_; }
  const index_t &frame_id() const { return frame_id_; }

  void set_reference(const TrajectoryReference &ref) override {
    reference_.set(ref);
  }

  void compute(const model_t &model, data_t &data, const vector_t &q,
               const vector_t &v) override;

  void compute_jacobian(const model_t &model, data_t &data,
                        const vector_t &q) override;

  void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                  const vector_t &q,
                                  const vector_t &v) override;

  void compute_error(const model_t &model, data_t &data, const vector_t &q,
                     const vector_t &v) override;

  Type type;

 private:
  string_t frame_;
  index_t frame_id_;
  matrix_t jacobian_full_;

  SE3TrajectoryReference reference_;
};

}  // namespace osc