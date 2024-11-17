#pragma once

#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>

#include "osc/common.hpp"

namespace osc {

class HolonomicConstraint {
   public:
    typedef double value_type;
    typedef std::size_t index_type;

    HolonomicConstraint(const index_type &dim, const index_type &model_nq,
                        const index_type &model_nv)
        : dimension_(dim), model_nq_(model_nq), model_nv_(model_nv) {}

    const index_type &dimension() const { return dimension_; }

    const index_type &model_nq() const { return model_nq_; }
    const index_type &model_nv() const { return model_nv_; }

    virtual eigen_matrix_sym_t constraint_jacobian(
        const model_sym_t &model, const eigen_vector_sym_t &q) const = 0;

   private:
    index_type dimension_;
    index_type model_nq_;
    index_type model_nv_;
};

}  // namespace osc