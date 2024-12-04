#include "osc/cost.hpp"

namespace osc {

bopt::quadratic_cost<double>::shared_ptr WeightedSumOfSquaresCost::to_cost(const model_t &model) const {
    vector_sym_t x = create_symbolic_vector("x", sz_);
    sym_t cost = x.dot(x);

    return nullptr;
}

}  // namespace osc
