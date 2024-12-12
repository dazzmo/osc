#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/constraint/linear.hpp"
#include "osc/se3.hpp"
#include "osc/trajectory_reference.hpp"

namespace osc {

class ContactBase : public FrameSE3 {
 public:
  // Contact types
  enum class Type { Point, Wrench };

  ContactBase(const model_t &model, const std::string &frame_name,
              const Type &type = Type::Point)
      : FrameSE3(model, frame_name) {
    // Set frame task type based on contact nature
    if (type == Type::Point) {
      FrameSE3::set_type(FrameSE3::Type::Position);
    } else {
      FrameSE3::set_type(FrameSE3::Type::Full);
    }

    // Default values for properties
    set_surface_normal(vector3_t::UnitZ());
    set_min_normal_force(0.0);
    set_max_normal_force(std::numeric_limits<double>::max());
    set_friction_coefficient(1.0);
  }

  const string_t &name() const { return name_; }
  void name(const string_t &name) { name_ = name; }

  const vector3_t &get_surface_normal() const { return n_; }
  void set_surface_normal(const vector3_t &n) { n_ = n; }

  const double &get_friction_coefficient() const { return mu_; }
  void set_friction_coefficient(const double &mu) { mu_ = mu; }

  const double &get_max_normal_force() const { return max_normal_force_; }
  void set_max_normal_force(const double &f) { max_normal_force_ = f; }

  const double &get_min_normal_force() const { return min_normal_force_; }
  void set_min_normal_force(const double &f) { min_normal_force_ = f; }

  const LinearConstraint &friction_cone_constraint() const {
    return *friction_cone_constraint_;
  }

  virtual void compute(const model_t &model, data_t &data, const vector_t &q,
                       const vector_t &v) = 0;

  void set_reference(const SE3TrajectoryReference &reference) {
    reference_ = reference;
  }

  const SE3TrajectoryReference &get_reference() const { return reference_; }

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

 protected:
  // Friction cone constraint
  std::unique_ptr<LinearConstraint> friction_cone_constraint_;
  // Contact surface normal vector (represented in world frame)
  vector3_t n_;
  // Friction coefficient
  double mu_;

  // Friction force upper bound
  double max_normal_force_;
  double min_normal_force_;

 private:
  string_t name_;

  string_t reference_frame_;
  index_t reference_frame_id_;

  SE3TrajectoryReference reference_;
};

}  // namespace osc

std::ostream &operator<<(std::ostream &os, osc::ContactBase const &c);