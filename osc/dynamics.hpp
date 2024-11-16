#pragma once
#include "osc/common.hpp"
#include "osc/contact.hpp"

namespace osc {

struct dynamics_constraint_handler {
    void add(const sym_t &jacobian, const sym_t &lambda) {}

    // Constraint Jacobian
    sym_t jacobian;
    // Constraint forces
    sym_t lambda;

    // Variables
    sym_t variables;
    // Parameters
    sym_t parameters;
};

template <typename VectorType>
struct dynamics_variables {
    VectorType a;
    VectorType u;
    VectorType lambda;
};

template <typename VectorType>
struct dynamics_parameters {
    VectorType q;
    VectorType v;
    VectorType additional;
};

class Dynamics {
   public:
    Dynamics(model_sym_t &model);

    // void register_contact_point(const ContactP &task);

    void register_actuation(const eigen_matrix_sym_t &B,
                            const eigen_vector_sym_t &u,
                            const eigen_vector_sym_t &p);

    void to_constraint();

    void add_constraint();

    void register_additional_dynamics(const sym_t &f,
                                      const std::vector<sym_t> &p);

    // dynamics_parameters<bopt::variable> parameters;
    // dynamics_parameters<double> parameters_d;

   private:
    dynamics_variables<eigen_vector_sym_t> variables_;
    dynamics_parameters<eigen_vector_sym_t> parameters_;
    eigen_vector_sym_t dynamics_;
};

}  // namespace osc