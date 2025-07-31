#pragma once

#include <pinocchio/algorithm/joint.hpp>

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

    State(const Model &model)
        : model_(model),
          data_(std::make_unique<ModelData>(model)),
          q_(pinocchio::neutral(model)),
          v_(Vector::Zero(model.nv)),
          jacobian_(Matrix6x::Zero(6, model_.nv)) {}

    Size nq() const { return model_.nq; }
    Size nv() const { return model_.nv; }

    const ConfigVectorType &q() const { return q_; }
    const TangentVectorType &v() const { return v_; }

    void update(const ConfigVectorType &q, const TangentVectorType &v) {}

    pinocchio::JointIndex getJointIndex(const String &joint) const;
    pinocchio::FrameIndex getFrameIndex(const String &frame) const;

    SE3 getTransformFrameToWorld(const String &frame) const;

    const Matrix6x &getJointJacobian(
        const pinocchio::JointIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const;

    const Matrix6x &getFrameJacobian(
        const pinocchio::FrameIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const;

    const Eigen::Vector3<Real> &getCentreOfMass() const;
    Matrix computeCentreOfMassJacobian() const;

    Matrix inertiaMatrix() const;
    Matrix inertiaMatrix() const;

   private:
    const Model &model_;
    std::unique_ptr<ModelData> data_;

    ConfigVectorType q_;
    TangentVectorType v_;

    /// @brief Work data for jacobian computations
    Matrix6x jacobian_;
};

}  // namespace osc
