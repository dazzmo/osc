#include "osc/cost.hpp"

#include "osc/osc.hpp"

namespace osc {

void EffortSquaredCost::add_to_program(OSC &osc_program) const {
  const model_sym_t &model = OSCComponent::get_model(osc_program);
  bopt::mathematical_program<double> &program =
      OSCComponent::get_program(osc_program);
  const osc_variables &variables =
      OSCComponent::get_variables(osc_program);
  osc_parameters &parameters =
      OSCComponent::get_parameters(osc_program);

  vector_sym_t u = create_symbolic_vector("u", variables.u.size());
  // Create expression evaluators
  sym_t u_s = casadi::eigen_to_casadi(u);
  sym_t cost = sym_t::dot(u_s, u_s);

  // Add to program
  program.add_quadratic_cost(
      bopt::casadi::quadratic_cost<value_type>::create(cost, u_s,
                                                       sym_vector_t({})),
      // Variables
      {variables.u},
      // Parameters
      {});
}

}  // namespace osc
