#include "osc/dynamics.hpp"

#include <pinocchio/algorithm/rnea.hpp>

namespace osc {

Dynamics::Dynamics(model_sym_t &model) {
    // Compute the inverse dynamics of the system
    eigen_vector_sym_t tau;

    pinocchio::DataTpl<sym_t> data(model);

    tau = pinocchio::rnea(model, data, parameters_.q, parameters_.v,
                          variables_.a);

    // Start with M \ddot q + C(q, \dot q) \dot q + G(q)
    dynamics_ = tau;
}

void Dynamics::register_contact_point(const ContactTask &task) {
    // Compute the Jacobian of the contact frame

    sym_t jac;
    sym_t lam;

    dynamics -= sym_t::mtimes(jac.T(), lam);
}

void Dynamics::register_actuation(const sym_t &B, const sym_t &u,
                                  const std::vector<sym_t> &p) {
    // Set control variables
    // variables_.u = u;

    // Add B(q) u
    dynamics -= sym_t::mtimes(B, u);
}

void Dynamics::register_additional_dynamics(const sym_t &f,
                                            const std::vector<sym_t> &p) {}

void Dynamics::to_constraint() {
    // Dynamics are created

    // Create linear constraint

    // Create vector
    sym_t x = sym_t::vertcat({variables_.a, variables_.u, variables_.lambda});
    sym_vector_t p = {};
    p.push_back(parameters_.q);
    p.push_back(parameters_.v);
    p.insert(p.end(), parameters_.other.begin(), parameters_.other.end());

    bopt::casadi::linear_constraint<double>::create(dynamics, x, p,
                                                    bopt::bound_type::Equality);
}

void Dynamics::add_constraint_to_program() {
    // program.add_linear_constraint(to_constraint(), {}, {});
}

}  // namespace osc