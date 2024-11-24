#include "osc/cost.hpp"
#include "osc/osc.hpp"

namespace osc
{



    void EffortSquaredCost::add_to_program(const model_sym_t &model, OSC &osc_program) {
        eigen_vector_sym_t u =
            create_symbolic_vector("u", osc_program.variables.u.size());
        // Create expression evaluators
        sym_t u_s = eigen_to_casadi<sym_elem_t>::convert(u);
        sym_t cost = sym_t::dot(u_s, u_s);

        // Add to program
        osc_program.program.add_quadratic_cost(
            bopt::casadi::quadratic_cost<value_type>::create(cost, u_s,
                                                             sym_vector_t({})),
            // Variables
            {eigen_to_std_vector<bopt::variable>::convert(
                osc_program.variables.u)},
            // Parameters
            {});
}

    
} // namespace osc
