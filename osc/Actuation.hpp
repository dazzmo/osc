#pragma once

#include "osc/State.hpp"
#include "osc/Types.hpp"

namespace osc {

/**
 * @brief Transforms control inputs u into generalised forces.
 *
 */
class ActuationAbstract {
   public:
    using SharedPtr = std::shared_ptr<ActuationAbstract>;

    /**
     * @brief The number of inputs associated with the actuation
     *
     * @return * Size
     */
    Size numInputs() const { return n_in_; }

    /**
     * @brief The number of outputs from the actuation mapping
     *
     * @return Size
     */
    Size numOutputs() const { return n_out_; }

    virtual Vector compute(const State &state,
                           const Eigen::Ref<const Vector> &u) const = 0;

    virtual void computeJacobian(const State &state,
                                 Eigen::Ref<Matrix> jac) const = 0;

   protected:
    ActuationAbstract(const Size &n_in, const Size &n_out)
        : n_in_(n_in), n_out_(n_out) {}

   private:
    Size n_in_;
    Size n_out_;
};

class Actuation {};

}  // namespace osc