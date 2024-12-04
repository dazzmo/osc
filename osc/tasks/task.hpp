#pragma once

#include "osc/common.hpp"
#include "osc/holonomic.hpp"
namespace osc {

/**
 * @brief Abstract task used to represent a holonomic task of the form x = f(q)
 *
 */
class AbstractTask : public HolonomicExpression {
 public:
  AbstractTask() : HolonomicExpression() {}
  AbstractTask(const index_t &dimension) : HolonomicExpression(dimension) {}

  const string_t &name() const { return name_; }

  /**
   * @brief Priority of the task, with lower values indicating higher priority.
   *
   */
  index_t priority = 0;

  virtual void evaluate_error(const model_t &model, data_t &data, vector_t &e,
                              vector_t &e_dot) const = 0;

 private:
  string_t name_;
};

}  // namespace osc