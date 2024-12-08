#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/constraint/linear.hpp"
#include "osc/holonomic.hpp"

namespace osc {

class AbstractContact : public HolonomicConstraint {
 public:
  AbstractContact(const model_t &model, const string_t &frame,
                  const index_t &dimension)
      : HolonomicConstraint(), frame_(frame) {
    // Ensure the frame exists on the model
    frame_id_ = model.getFrameId(frame);
    if (frame_id_ == model.frames.size()) {
      assert("Model does not have specified frame");
    }

    // Create jacobian for contact
    jacobian_full_ = matrix_t::Zero(6, model.nv);
  }

  virtual void compute(const model_t &model, data_t &data, const vector_t &q,
                       const vector_t &v) = 0;

  const string_t &name() const { return name_; }
  void name(const std::string &name) { name_ = name; }

  const string_t &frame() const { return frame_; }
  const index_t &frame_id() const { return frame_id_; }

  /**
   * @brief Contact force or wrench experienced at the provided contact
   *
   */
  vector_t force;

 protected:
  matrix_t jacobian_full_;

 private:
  string_t name_;
  // Frame for the contact to occur
  string_t frame_;
  index_t frame_id_;
};

class AbstractFrictionContact : public AbstractContact {
 public:
  AbstractFrictionContact(const model_t &model, const string_t &frame,
                          const index_t &dimension)
      : AbstractContact(model, frame, dimension) {
    set_friction_coefficient(1.0);
    set_surface_normal(vector3_t::UnitZ());
    set_min_normal_force(0.0);
    set_max_normal_force(std::numeric_limits<double>::max());
    friction_cone_constraint_ = std::make_unique<LinearConstraint>(
        4, 3, LinearConstraint::Type::Inequality);
  }

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

 protected:
  std::unique_ptr<LinearConstraint> friction_cone_constraint_;

  // Contact surface normal vector (represented in world frame)
  vector3_t n_;
  // Friction coefficient
  double mu_;

  // Friction force upper bound
  double max_normal_force_;
  double min_normal_force_;
};

class FrictionContact3D : public AbstractFrictionContact {
 public:
  FrictionContact3D(const model_t &model, const string_t &frame)
      : AbstractFrictionContact(model, frame, 3) {
    friction_cone_constraint_ = std::make_unique<LinearConstraint>(4, 3);
  }

  index_t dim() const override { return 3; }

  void compute(const model_t &model, data_t &data, const vector_t &q,
               const vector_t &v) override;

  void compute_jacobian(const model_t &model, data_t &data,
                        const vector_t &q) override;

  void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                  const vector_t &q,
                                  const vector_t &v) override;
};

// std::ostream &operator<<(std::ostream &os, AbstractContact const &m) {
//   return os << "Contact: " << m.name() << '\n'
//             << "reference frame : " << m.reference_frame() << '\n'
//             << "e : " << m.get_error().transpose() << '\n'
//             << "e_dot : " << m.get_error_dot().transpose() << '\n'
//             << "Kp : " << m.Kp().transpose() << '\n'
//             << "Kd : " << m.Kd().transpose() << '\n';
// }

}  // namespace osc