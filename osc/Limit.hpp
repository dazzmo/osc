#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

class LimitAbstract {
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

    void toQPObjective(const State &state, Eigen::Ref<Matrix> H,
                       Eigen::Ref<Vector> g) {
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

    void toQPConstraint(const State &state, Eigen::Ref<Matrix> A,
                        Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA,
                        Eigen::Ref<Vector> ubx, Eigen::Ref<Vector> lbx) {
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
    LimitAbstract() : dimension_(0) {}
    LimitAbstract(const Size &dimension) : dimension_(dimension) {}

    void setDimension(const Size &dimension) { dimension_ = dimension; }

   private:
    Size dimension_;
};

template <typename T>
class MotionLimit {};

template <typaname T>
class ActuationLimit {};

};  // namespace osc