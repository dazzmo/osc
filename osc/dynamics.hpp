#pragma once
#include "osc/common.hpp"
#include "osc/contact.hpp"

namespace osc {

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
};

class Dynamics {
    friend class OSC;

   public:
    typedef double value_type;

    Dynamics(model_sym_t &model);

    void register_actuation(const eigen_matrix_sym_t &B,
                            const eigen_vector_var_t &u_v);

    void add_constraint(const HolonomicConstraint &constraint);

    bopt::linear_constraint<value_type>::shared_ptr create_linear_constraint();

   private:
    model_sym_t &model;

    dynamics_variables<eigen_vector_sym_t> variables_s_;
    dynamics_parameters<eigen_vector_sym_t> parameters_s_;

    eigen_vector_sym_t dynamics_;
};

}  // namespace osc