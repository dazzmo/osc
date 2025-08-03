#pragma once

#include "osc/Fwd.hpp"

namespace osc {

/**
 * @brief Handler class for all kinodynamic measurements and evaluations of the
 * robot for a given control problem.
 *
 */
class State {
   public:
    using ConfigVectorType = typename Model::ConfigVectorType;
    using TangentVectorType = typename Model::TangentVectorType;
    using Matrix3x = Eigen::Matrix<Real, 3, Eigen::Dynamic>;
    using Matrix6x = Eigen::Matrix<Real, 6, Eigen::Dynamic>;

    State() = default;
    ~State() = default;

    State(const Model &model);

    Size nq() const { return model_.nq; }
    Size nv() const { return model_.nv; }

    const ConfigVectorType &q() const { return q_; }
    const TangentVectorType &v() const { return v_; }

    const Model &model() const { return model_; }
    bool hasFloatingBase() const { return model_.existJointName("root_joint"); }

    void update(const ConfigVectorType &q, const TangentVectorType &v);

    pinocchio::JointIndex getJointIndex(const String &joint) const;
    pinocchio::FrameIndex getFrameIndex(const String &frame) const;

    SE3 getTransformFrameToWorld(const String &frame) const;

    const Matrix6x &computeJointJacobian(
        const pinocchio::JointIndex &index) const;

    const Matrix6x &computeFrameJacobian(
        const pinocchio::FrameIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const;

    Motion getFrameVelocity(const pinocchio::FrameIndex &index,
                            const pinocchio::ReferenceFrame &reference_frame =
                                pinocchio::LOCAL) const;

    Motion getFrameClassicalAcceleration(
        const pinocchio::FrameIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const;

    const Vector3 &getCentreOfMass() const;
    const Vector3 &getCentreOfMassVelocity() const;
    const Vector3 &getCentreOfMassAcceleration() const;
    const Matrix3x &computeCentreOfMassJacobian() const;

    const Matrix &getInertiaMatrix() const;
    Vector computeNonlinearEffects(const Eigen::Ref<const Vector> &q,
                                   const Eigen::Ref<const Vector> &v) const;

   private:
    const Model &model_;
    std::unique_ptr<ModelData> data_;

    ConfigVectorType q_;
    TangentVectorType v_;

    /// @brief Work data for jacobian computations
    Matrix6x jacobian_;
};

}  // namespace osc
