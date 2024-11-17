#include "osc/dynamics.hpp"

#include <pinocchio/algorithm/rnea.hpp>

namespace osc {

Dynamics::Dynamics(model_sym_t &model) : model(model) {
    // Compute the inverse dynamics of the system
    eigen_vector_sym_t tau;

    pinocchio::DataTpl<sym_t> data(model);

    // Create variables
    parameters_s_.q = create_symbolic_vector("q", model.nq);
    parameters_s_.v = create_symbolic_vector("v", model.nv);
    variables_s_.a = create_symbolic_vector("a", model.nv);

    // Start with M \ddot q + C(q, \dot q) \dot q + G(q)
    dynamics_ = pinocchio::rnea(model, data, parameters_s_.q, parameters_s_.v,
                                variables_s_.a);
}

bopt::linear_constraint<Dynamics::value_type>::shared_ptr
Dynamics::to_constraint() {
    // Create expression evaluators
    sym_t q_s = eigen_to_casadi<sym_elem_t>::convert(parameters_s_.q);
    sym_t v_s = eigen_to_casadi<sym_elem_t>::convert(parameters_s_.v);
    sym_t a_s = eigen_to_casadi<sym_elem_t>::convert(variables_s_.a);

    sym_t u_s = eigen_to_casadi<sym_elem_t>::convert(variables_s_.u);

    sym_t f_s = eigen_to_casadi<sym_elem_t>::convert(variables_s_.lambda);

    sym_t dyn_s = eigen_to_casadi<sym_elem_t>::convert(dynamics_);

    // todo - need to consider the frame transformation from contact frame to
    // todo - end-effector frame

    // Create vector for linearisation
    sym_t x = sym_t::vertcat(sym_vector_t({a_s, u_s, f_s}));

    return bopt::casadi::linear_constraint<value_type>::create(
        dyn_s, x, sym_vector_t({q_s, v_s}));
}

void Dynamics::register_actuation(const eigen_matrix_sym_t &B,
                                  const eigen_vector_var_t &u_v) {
    eigen_vector_sym_t u = create_symbolic_vector("u", u_v.size());

    // Add B(q) u
    dynamics_ -= B * u;

    // Set control variables
    variables_s_.u = u;

    // Add variables
    variables_v_.u = u_v;
}

void Dynamics::add_constraint(const ContactPoint &contact,
                              const eigen_vector_var_t &lambda_v) {
    // Add to dynamics
    auto J = contact.contact_jacobian(model, parameters_s_.q);

    eigen_vector_sym_t lambda =
        create_symbolic_vector("lambda", lambda_v.size());

    dynamics_ -= J.topRows(3).transpose() * lambda;

    variables_s_.lambda.conservativeResize(variables_s_.lambda.size() +
                                           lambda.size());
    variables_s_.lambda.bottomRows(lambda.size()) << lambda;

    // Add variables to the variable list
    variables_v_.lambda.conservativeResize(variables_v_.lambda.size() +
                                           lambda_v.size());
    variables_v_.lambda.bottomRows(lambda_v.size()) << lambda_v;
}

}  // namespace osc