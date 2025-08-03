#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

class ConstraintAbstract {
   public:
    Size getDimension() const { return dimension_; }

    /**
     * @brief Set the effective gain of the task
     *
     * @param gain
     */
    void setGain(const Real &gain) { gain_ = gain; }
    const Real &getGain() const { return gain_; }

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

    Vector compute(const State &state) {
        Vector c(getDimension());
        jac.setZero();
        computeError(state, c);
        return c;
    }

    Matrix computeJacobian(const State &state) {
        Matrix jac(getDimension(), state.nv());
        jac.setZero();
        computeJacobian(state, jac);
        return jac;
    }


    // todo - virtual casadi::Function toCasadiFunction() const = 0;

   protected:
    ConstraintAbstract() : dimension_(0) {}
    ConstraintAbstract(const Size &dimension) : dimension_(dimension) {}

    void setDimension(const Size &dimension) { dimension_ = dimension; }

   private:
    Size dimension_;
    Real gain_;
};

};  // namespace osc