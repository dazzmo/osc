#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

class TaskAbstract {
   public:
    using SharedPtr = std::shared_ptr<TaskAbstract>;
    /**
     * @brief Returns the dimension of the task.
     * @note This is the dimension of the task error, not necessarily the
     * dimension of the task. For the dimension of the task see
     * getTaskDimension()
     *
     * @return Size
     */
    Size getDimension() const { return dimension_; }
    Size getTaskDimension() const { return task_dimension_; }

    /**
     * @brief Set the effective gain of the task
     * 
     * @param gain 
     */
    void setGain(const Real &gain) { gain_ = gain; }
    const Real &getGain() const { return gain_; }

    const Vector &getWeighting() { return weighting_; }
    void setWeighting(const Eigen::Ref<Vector> &weighting) {
        weighting_ = weighting;
    }
    void setWeighting(const Real &weighting) {
        weighting_.setConstant(weighting);
    }

    void getVariables() {}

    /**
     * @brief Computes the task error
     *
     * @param state
     * @param e
     */
    virtual void computeError(const State &state, Eigen::Ref<Vector> e) = 0;

    /**
     * @brief Computes the task Jacobian
     *
     * @param state
     * @param jac
     */
    virtual void computeJacobian(const State &state,
                                 Eigen::Ref<Matrix> jac) = 0;
    /**
     * @brief Computes the task acceleration bias term, given as \gamma =
     * \dot{J}(q) \dot{q}
     *
     * @param state
     * @param jac
     */
    virtual void computeAccelerationBias(const State &state,
                                         Eigen::Ref<Matrix> bias) = 0;

    Vector computeError(const State &state) {
        Vector e(getDimension());
        jac.setZero();
        computeError(state, e);
        return e;
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

        // H = gain_ * 2.0 * A.transpose() * A;
        // g = gain_ *  A.transpose() * b;
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

    // todo - virtual casadi::Function toCasadiFunction(const Model &model) const = 0;

    void setErrorPD(const Vector &Kp, const Vector &Kd) {}

   protected:
    TaskAbstract() : dimension_(0), task_dimension_(0) {}
    TaskAbstract(const Size &dimension)
        : dimension_(dimension), task_dimension_(dimension) {}

    void setDimension(const Size &dimension) { dimension_ = dimension; }
    void setTaskDimension(const Size &dimension) {
        task_dimension_ = dimension;
    }

   private:
    Size dimension_;
    Size task_dimension_;
    Vector weighting_;
};

template <typename _TargetType>
class Task : public TaskAbstract {
   public:
    using TargetType = _TargetType;

    Task(const VariableVector &variables) {}

    void setTarget(const TargetType &target) { target_ = target; }
    const TargetType &getTarget() const { return target_; }

   private:
    TargetType target_;
};

template<typename _TargetType>
class MotionTask : public TaskAbstract {

};

template<typename _TargetType>
class ActuationTask : public TaskAbstract {

};



};  // namespace osc