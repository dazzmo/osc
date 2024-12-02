#pragma once

#include <Eigen/Core>
#include <bopt/constraints.hpp>
#include <bopt/program.hpp>
#include <bopt/variable.hpp>

#include "osc/common.hpp"

namespace osc {

class AbstractLinearConstraint {
 public:
  virtual std::shared_ptr<bopt::linear_constraint<double>> to_constraint(
      const model_t &model) const = 0;
};

}  // namespace osc
