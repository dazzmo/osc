#pragma once

#include <bopt/costs.hpp>

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
  virtual bopt::quadratic_cost<double>::shared_ptr to_cost(
      const model_t &model) const = 0;

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

  bopt::quadratic_cost<double>::shared_ptr to_cost(
      const model_t &model) const override;

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