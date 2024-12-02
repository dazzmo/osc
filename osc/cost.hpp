#pragma once

#include <bopt/program.hpp>

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>
#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/common.hpp"

namespace osc {

class OSC;

/**
 * @brief Abstract quadratic cost with interface method to generate a quadratic
 * cost expression.
 *
 */
class AbstractQuadraticCost {
 public:
  virtual bopt::quadratic_cost<double>::shared_ptr to_cost() = 0;

 private:
};


/**
 * @brief Weighted quadratic cost of the form \f$ ||x||_w^2 \f$
 * 
 */
class WeightedSumOfSquaresCost : public AbstractQuadraticCost {
 public:
  WeightedSumOfSquaresCost(const index_t &sz) : sz_(sz) {
    parameters.w = bopt::create_variable_vector("w", sz);
  }

  bopt::quadratic_cost<double>::shared_ptr to_cost() override;

  struct parameters {
    std::vector<bopt::variable> w;
  };

  // Weighting
  vector_t w;

  parameters parameters;

 private:
  index_t sz_ = 0;
};

}  // namespace osc