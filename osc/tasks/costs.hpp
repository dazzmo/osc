#pragma once

#include "osc/cost.hpp"
#include "osc/pid.hpp"

namespace osc {

class AbstractTaskCost : public AbstractQuadraticCost {
 public:
  AbstractTaskCost(const model_t &model,
                   const std::shared_ptr<AbstractTask> &task)
      : AbstractQuadraticCost(), task_(task) {}

  /**
   * @brief The task the cost is associated with
   *
   * @return std::shared_ptr<AbstractTask>&
   */
  std::shared_ptr<AbstractTask> &task() { return task_; }

 protected:
  std::shared_ptr<AbstractTask> task_;
};

/**
 * @brief Weighted task cost of the form \f$ || \ddot x - \ddot x_d ||_w^2 \f$
 *
 */
class WeightedTaskCost : public AbstractTaskCost {
 public:
  WeightedTaskCost(const model_t &model,
                   const std::shared_ptr<AbstractTask> &task);

  struct parameters {
    std::vector<bopt::variable> w;
    std::vector<bopt::variable> x;
  };

  parameters parameters;

  // PID tracking gains
  pid_gains<double> pid;

  // Weighting
  vector_t w;
  // Desired task second derivative
  vector_t x;

  bopt::quadratic_cost<double>::shared_ptr to_cost(
      const model_t &model) const override;
};

class HeirarchicalTaskCost {
  // todo - custom qudratic cost
  // class Cost : public bopt::quadratic_cost<double> {};
};
}  // namespace osc