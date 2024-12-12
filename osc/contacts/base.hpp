#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/constraint/linear.hpp"
#include "osc/tasks/frame.hpp"

namespace osc {

class ContactBase : public FrameTask {
 public:
  // Contact types
  enum class Type { Point, Wrench };

  ContactBase(const model_t &model, const std::string &frame_name,
              const Type &type = Type::Point,
              const std::string &reference_frame = "universe")
      : FrameTask(model, frame_name) {
    // Set frame task type based on contact nature
    if (type == Type::Point) {
      FrameTask::set_type(FrameTask::Type::Position);
    } else {
      FrameTask::set_type(FrameTask::Type::Full);
    }

    // Default values for properties
    set_surface_normal(vector3_t::UnitZ());
    set_min_normal_force(0.0);
    set_max_normal_force(std::numeric_limits<double>::max());
    set_friction_coefficient(1.0);
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

  virtual void compute(const model_t &model, data_t &data, const vector_t &q,
                       const vector_t &v) = 0;

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
};

}  // namespace osc

std::ostream &operator<<(std::ostream &os, osc::ContactBase const &c);