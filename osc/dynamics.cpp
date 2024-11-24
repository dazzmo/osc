#include "osc/dynamics.hpp"
#include "osc/osc.hpp"

#include <pinocchio/algorithm/rnea.hpp>

namespace osc {

Dynamics::Dynamics(const model_t &model) : model(model.cast<sym_t>()) {
    // Compute the inverse dynamics of the system
    eigen_vector_sym_t tau;

    pinocchio::DataTpl<sym_t> data(this->model);

    // Create variables
    parameters_.q = create_symbolic_vector("q", model.nq);
    parameters_.v = create_symbolic_vector("v", model.nv);
    variables_.a = create_symbolic_vector("a", model.nv);
    variables_.lambda.resize(0);
    variables_.u.resize(0);

    // Start with M \ddot q + C(q, \dot q) \dot q + G(q)
    dynamics_ = pinocchio::rnea(this->model, data, parameters_.q, parameters_.v,
                                variables_.a);
}

void Dynamics::register_actuation(const eigen_matrix_sym_t &B,
                                  const eigen_vector_var_t &u_v) {
    eigen_vector_sym_t u = create_symbolic_vector("u", u_v.size());

    // Add B(q) u
    dynamics_ -= B * u;

    // Set control variables
    variables_.u = u;
}

void Dynamics::add_constraint_forces(const eigen_vector_sym_t &lambda) {
    variables_.lambda.conservativeResize(variables_.lambda.size() +
                                         lambda.size());
    variables_.lambda.bottomRows(lambda.size()) << lambda;
}

void Dynamics::add_constraint(const HolonomicConstraint &constraint) {
    // Add to dynamics
    auto J = constraint.constraint_jacobian(model, parameters_.q);

    eigen_vector_sym_t lambda =
        create_symbolic_vector("lambda", constraint.dimension());

    add_constraint_forces(lambda);

    dynamics_ -= J.transpose() * lambda;
}

void Dynamics::add_to_program(const model_sym_t &model, OSC &osc_program) {
    // Create expression evaluators
    sym_t q_s = eigen_to_casadi<sym_elem_t>::convert(parameters_.q);
    sym_t v_s = eigen_to_casadi<sym_elem_t>::convert(parameters_.v);

    sym_t a_s = eigen_to_casadi<sym_elem_t>::convert(variables_.a);
    sym_t u_s = eigen_to_casadi<sym_elem_t>::convert(variables_.u);
    sym_t f_s = eigen_to_casadi<sym_elem_t>::convert(variables_.lambda);

    sym_t dyn_s = eigen_to_casadi<sym_elem_t>::convert(dynamics_);

    // todo - need to consider the frame transformation from contact frame to
    // todo - end-effector frame

    // Create vector for linearisation
    sym_t x = sym_t::vertcat(sym_vector_t({a_s, u_s, f_s}));

    // Create variable vector
    eigen_vector_var_t xv(osc_program.variables.a.size() +
                          osc_program.variables.u.size() +
                          osc_program.variables.lambda.size());

    xv << osc_program.variables.a, osc_program.variables.u,
        osc_program.variables.lambda;

    osc_program.program.add_linear_constraint(
        bopt::casadi::linear_constraint<value_type>::create(
            dyn_s, x, sym_vector_t({q_s, v_s})),
        // Variables
        {eigen_to_std_vector<bopt::variable>::convert(xv)},
        // Parameters
        {
            eigen_to_std_vector<bopt::variable>::convert(
                osc_program.parameters.q),
            eigen_to_std_vector<bopt::variable>::convert(
                osc_program.parameters.v),
        });
}

}  // namespace osc