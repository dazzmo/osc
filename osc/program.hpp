#pragma once
#include "osc/common.hpp"
#include <bopt/program.hpp>

namespace osc {

/**
 * @brief A component of an operational space controller, such as constraint,
 * cost or task
 *
 */
class OSCComponent {
 public:
 protected:
  template <typename ProblemType>
  const model_sym_t &get_model(ProblemType &problem) const {
    return problem.model_sym;
  }

  template <typename ProblemType>
  osc_variables &get_variables(
      ProblemType &problem) const {
    return problem.variables_;
  }

  template <typename ProblemType>
  osc_parameters &get_parameters(ProblemType &problem) const {
    return problem.parameters_;
  }

  template <typename ProblemType>
  bopt::mathematical_program<double> &get_program(ProblemType &problem) const {
    return problem.program_;
  }

 private:
};

}  // namespace osc