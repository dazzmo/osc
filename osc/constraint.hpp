#pragma once

#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>

#include "osc/common.hpp"

namespace osc {

class OSC;

/**
 * @brief Holonomic constraint related to a model
 *
 */
class HolonomicConstraint {
    friend class OSC;
   public:
    typedef double value_type;
    typedef std::size_t index_type;

    HolonomicConstraint(const index_type &dim) : dimension_(dim) {}

    const index_type &dimension() const { return dimension_; }

    virtual eigen_matrix_sym_t constraint_jacobian(
        const model_sym_t &model, const eigen_vector_sym_t &q) const = 0;

   protected:
    virtual void add_to_program(const model_sym_t &model, OSC &osc) = 0;

   private:
    index_type dimension_;
};

}  // namespace osc