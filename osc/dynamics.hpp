#pragma once
#include "osc/common.hpp"
#include "osc/contact.hpp"
#include "osc/constraint.hpp"

namespace osc {

class OSC;

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

// Forward declaration
class OSC;

class Dynamics {
    friend class OSC;

   public:
    typedef double value_type;

    Dynamics(const model_t &model);

    void register_actuation(const eigen_matrix_sym_t &B,
                            const eigen_vector_var_t &u_v);

    void add_constraint(const HolonomicConstraint &constraint);

    void add_to_program(const model_sym_t &model, OSC &osc_program);

   private:
    model_sym_t model;

    dynamics_variables<eigen_vector_sym_t> variables_;
    dynamics_parameters<eigen_vector_sym_t> parameters_;

    eigen_vector_sym_t dynamics_;

    void add_constraint_forces(const eigen_vector_sym_t &lambda);
};

}  // namespace osc