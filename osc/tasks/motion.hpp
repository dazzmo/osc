
#pragma once

#include "osc/se3.hpp"
#include "osc/tasks/base.hpp"
#include "osc/trajectory_reference.hpp"

namespace osc {

/**
 * @brief Tasks associated with motion tracking, and PD-based error correction
 *
 */
class MotionTask : public TaskBase {
 public:
  enum class Type { Position = 0, Orientation, Full };

  MotionTask(const model_t &model) {}

  virtual void set_reference(const TrajectoryReference &reference) = 0;

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

  const vector_t get_error() const { return e_; }
  const vector_t get_error_dot() const { return e_dot_; }

  /**
   * @brief Returns the desired task acceleration, typically of the form Kp e +
   * Kd e_dot + xacc_d
   *
   * @return const vector_t&
   */
  const vector_t &get_desired_acceleration() const { return xacc_des_; }

  const matrix_t &jacobian() const { return jacobian_; }
  const vector_t &jacobian_dot_q_dot() const { return jacobian_dot_q_dot_; }

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
  virtual void compute_jacobian(const model_t &model, data_t &data,
                                const vector_t &q) = 0;

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
  virtual void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                          const vector_t &q,
                                          const vector_t &q_dot) = 0;

  virtual void compute_error(const model_t &model, data_t &data,
                             const vector_t &q, const vector_t &v) = 0;

 protected:
  matrix_t jacobian_;
  vector_t jacobian_dot_q_dot_;

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