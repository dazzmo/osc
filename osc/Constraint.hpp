#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

class ConstraintAbstract {
   public:
    Size getDimension() const { return dimension_; }

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

    void toQPConstraint(const State &state, Eigen::Ref<Matrix> A,
                        Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA) {
        const Vector e = computeError(state);
        const Matrix J = computeJacobian(state);

        // Compute the desired task acceleration to minimise the error
        // const Vector ad = computeDesiredTaskAcceleration(e);

        // const Matrix &A = J;
        // const Vector b = bias - ad;
        // // Compute weighting

        // H = 2.0 * A.transpose() * A;
        // g = A.transpose() * b;
    }
    // todo - virtual casadi::Function toCasadiFunction() const = 0;

   protected:
    ConstraintAbstract() : dimension_(0) {}
    ConstraintAbstract(const Size &dimension) : dimension_(dimension) {}

    void setDimension(const Size &dimension) { dimension_ = dimension; }

   private:
    Size dimension_;
};

};  // namespace osc