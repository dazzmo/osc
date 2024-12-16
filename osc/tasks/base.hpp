#pragma once

#include "osc/common.hpp"
namespace osc {

/**
 * @brief Abstract task used to represent a holonomic task of the form x = f(q)
 *
 */
class TaskBase {
 public:
  TaskBase() {}

  const string_t &name() const { return name_; }
  void name(const string_t &name) { name_ = name; }

  /**
   * @brief Dimension of the task expression
   *
   * @return index_t
   */
  virtual index_t dim() const = 0;

  /**
   * @brief Compute all quantities associated with the given task
   *
   * @param model
   * @param data
   * @param q
   * @param v
   */
  virtual void compute(const model_t &model, data_t &data, const vector_t &q,
                       const vector_t &v) = 0;

 private:
  string_t name_;
};

}  // namespace osc