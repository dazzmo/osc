
#pragma once

#include "osc/tasks/base.hpp"
#include "osc/trajectory_reference.hpp"

namespace osc {

/**
 * @brief Tasks associated with motion tracking, and PD-based error correction
 *
 */
class MotionTask : public TaskBase, public HolonomicConstraint {
 public:
  enum class Type { Position = 0, Orientation, Full };

  MotionTask(const model_t &model, const string_t &reference_frame = "universe")
      : TaskBase(), HolonomicConstraint(), reference_frame_(reference_frame) {
    reference_frame_id_ = model.getFrameId(reference_frame);
    if (model.getFrameId(reference_frame) == model.frames.size()) {
      assert("Model does not have specified reference frame");
    }
  }

  virtual void set_reference(const TrajectoryReference &reference) = 0;

  virtual void compute_error(const model_t &model, data_t &data,
                             const vector_t &q, const vector_t &v) = 0;

  const std::string &reference_frame() const { return reference_frame_; }
  const index_t &reference_frame_id() const { return reference_frame_id_; }

  /**
   * @brief Proportional gains for task tracking
   *
   * @return const vector_t&
   */
  const vector_t &Kp() const { return Kp_; }

  /**
   * @brief Derivative gains for task derivative tracking
   *
   * @return const vector_t&
   */
  const vector_t &Kd() const { return Kd_; }

  /**
   * @brief Set the Kp gains
   *
   * @param Kp
   */
  void Kp(const vector_t &Kp) {
    assert(Kp.size() == dim() && "Incorrect Kp size");
    Kp_ = Kp;
  }

  /**
   * @brief Set the Kd gains
   *
   * @param Kd
   */
  void Kd(const vector_t &Kd) {
    assert(Kd.size() == dim() && "Incorrect Kd size");
    Kd_ = Kd;
  }

  /**
   * @brief Returns the desired task acceleration, typically of the form Kp e +
   * Kd e_dot + xacc_d
   *
   * @return const vector_t&
   */
  const vector_t &get_desired_acceleration() const { return xacc_des_; }

  const vector_t get_error() const { return e_; }
  const vector_t get_error_dot() const { return e_dot_; }

 protected:
  vector_t e_;
  vector_t e_dot_;

  vector_t xacc_des_;

  vector_t Kp_;
  vector_t Kd_;

 private:
  string_t reference_frame_;
  index_t reference_frame_id_;
};

}  // namespace osc

std::ostream &operator<<(std::ostream &os, osc::MotionTask const &m);