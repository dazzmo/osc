#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

class ConstraintAbstract {
   public:
    using SharedPtr = std::shared_ptr<ConstraintAbstract>;

    virtual Size numConstraints() const = 0;

    /**
     * @brief Set the effective gain of the task
     *
     * @param gain
     */
    void setGain(const Real &gain) { gain_ = gain; }
    const Real &gain() const { return gain_; }

    /**
     * @brief Computes the task error
     *
     * @param state
     * @param e
     */
    virtual void compute(const State &state, Eigen::Ref<Vector> c) = 0;

    /**
     * @brief Computes the task Jacobian
     *
     * @param state
     * @param jac
     */
    virtual void computeJacobian(const State &state,
                                 Eigen::Ref<Matrix> jac) = 0;

    // todo - virtual casadi::Function toCasadiFunction() const = 0;

   protected:
    ConstraintAbstract() : gain_(1.0) {}

   private:
    Real gain_;
};

}  // namespace osc