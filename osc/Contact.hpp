#pragma once

#include "osc/FrictionConeModel.hpp"
#include "osc/Fwd.hpp"
#include "osc/State.hpp"
#include "osc/Task.hpp"

namespace osc {

class ContactAbstract : public TaskAbstract {
   public:
    using SharedPtr = std::shared_ptr<ContactAbstract>;

    /**
     * @brief Set the surface normal of the contact point (within the world
     * frame)
     *
     * @param normal
     */
    void setSurfaceNormal(const Eigen::Vector3<Real> &normal) {
        normal_ = normal;
    }

    void setFrictionCoefficient(const Real &mu) { mu_ = mu; }

    /**
     * @brief Computes the contact jacobian that maps a world-frame force to the
     * generalised inputs
     *
     * @param state
     * @param jac
     */
    virtual void computeContactJacobian(const State &state,
                                        Eigen::Ref<Matrix> jacobian) = 0;

    /**
     * @brief Computes the transform from the world to the contact surface frame
     *
     */
    void worldToContactSurfaceTransform() {}

    void toQPConstraints(const State &state, Eigen::Ref<Matrix> A,
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

        // todo - Possibly include target force to reach
    }
    // todo - virtual casadi::Function toCasadiFunction() const = 0;

   protected:
    ContactAbstract() : dimension_(0), task_dimension_(0) {}
    ContactAbstract(const Size &dimension,
                    const FrictionConeModelAbstract::SharedPtr &friction_cone)
        : dimension_(dimension) {}

    void setDimension(const Size &dimension) { dimension_ = dimension; }

   private:
    String frame_;
    Vector3 normal_;
};

class PointContact : public ContactAbstract {
    static constexpr int DIMENSION = 3;

   public:
    PointContact(const String &frame,
                 FrictionConeModelAbstract::SharedPtr &friction_cone)
        : ContactAbstract(DIMENSION, friction_cone) {}

    void computeError(const State &state, Eigen::Ref<Vector> e) override {
        e = state.getTransformFrameToWorld(frame_).translation() - target_;
    }

    void computeJacobian(const State &state,
                         Eigen::Ref<Matrix> jacobian) override {
        jacobian = state.getFrameJacobian(state.getFrameIndex(frame_))
                       .topRows<DIMENSION>();
    }

    void computeAccelerationBias(const State &state,
                                 Eigen::Ref<Matrix> bias) override {
        bias = state.getClassicalFrameAcceleration(frame_).linear();
    }

    void computeContactJacobian(const State &state,
                                Eigen::Ref<Matrix> jacobian) override {
        jacobian =
            state
                .getFrameJacobian(state.getFrameIndex(frame_), pinocchio::WORLD)
                .topRows<DIMENSION>();
    }

   private:
    String &frame_;
    Vector3 target_;
};

// todo - wrench contact

};  // namespace osc