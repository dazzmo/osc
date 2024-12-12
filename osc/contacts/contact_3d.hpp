#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/constraint/linear.hpp"
#include "osc/contacts/base.hpp"
#include "osc/tasks/frame.hpp"

namespace osc {

class FrictionContact3D : public ContactBase {
 public:
  FrictionContact3D(const model_t &model, const string_t &frame)
      : ContactBase(model, frame, ContactBase::Type::Point) {
    friction_cone_constraint_ = std::make_unique<LinearConstraint>(
        6, 3, LinearConstraint::Type::Inequality);
  }

  virtual void compute(const model_t &model, data_t &data, const vector_t &q,
                       const vector_t &v) override;

 protected:
};

}  // namespace osc