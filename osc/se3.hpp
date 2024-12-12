#pragma once

#include "osc/common.hpp"

namespace osc {

class FrameSE3 {
 public:
  enum class Type { 
    // Positional frame (orientation not accounted for)
    Position = 0, 
    // Orientation frame (position not accounted for)
    Orientation, 
    // SE3 frame (position and orientation coupled together)
    Full };

  FrameSE3(const model_t &model, const std::string &frame_name,
           const Type &type = Type::Full);

  index_t dim() const;

  const string_t &frame() const { return frame_; }
  const index_t &frame_id() const { return frame_id_; }

  void set_type(const Type &type);
  const Type &get_type() const { return type_; }

  void set_reference_frame(const model_t &model, const string_t &frame) {
    reference_frame_ = frame;
    reference_frame_id_ = model.getFrameId(reference_frame_);
    if (reference_frame_id_ == model.frames.size()) {
      assert("Model does not have specified reference frame");
    }
  }

  const string_t &reference_frame() const { return reference_frame_; }
  const index_t &reference_frame_id() const { return reference_frame_id_; }

  /**
   * @brief Whether to use the local frame for computations or use a
   * world-aligned frame.
   *
   * @param flag
   */
  void use_local_frame(const bool &flag) { use_local_frame_ = flag; }

  void compute_jacobian(const model_t &model, data_t &data, matrix_t &jacobian);

  void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                  vector_t &jacobian_dot_q_dot);

  void compute_error(const model_t &model, data_t &data,
                     const pinocchio::SE3 &p_ref,
                     const pinocchio::Motion &v_ref, vector_t &e,
                     vector_t &e_dot);

 private:
  Type type_;

  bool use_local_frame_;

  string_t frame_;
  string_t reference_frame_;

  index_t frame_id_;
  index_t reference_frame_id_;

  matrix_t jacobian_full_;
};

}  // namespace osc
