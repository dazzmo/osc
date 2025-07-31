#pragma once

#include "osc/State.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace osc {

State::State(const Model &model)
    : model_(model), data_(std::make_unique<ModelData>(model)) {}

void State::update(const ConfigVectorType &q, const TangentVectorType &v) {
    q_ = q;
    v_ = v;
    pinocchio::framesForwardKinematics(model_, *data_, q, v);
    pinocchio::computeJointJacobians(model_, *data_);
}

pinocchio::JointIndex State::getJointIndex(const String &joint) const {
    const auto index = model_.getJointId(joint);
    assert(index <= model_.joints.size() && "Joint does not exist");
    return index;
}
pinocchio::FrameIndex State::getFrameIndex(const String &frame) const {
    const auto index = model_.getFrameId(joint);
    assert(index <= model_.frames.size() && "Frame does not exist");
    return index;
}

SE3 State::getTransformFrameToWorld(const String &frame) const {
    return data_->oMf[getFrameIndex(frame)];
}

const State::Matrix6x &State::getJointJacobian(
    const pinocchio::JointIndex &index,
    const pinocchio::ReferenceFrame &reference_frame) const {
    pinocchio::computeJointJacobian(model_, *data_, q_, index, referemce_frame,
                                    jacobian_);
    return jacobian_;
}

const State::Matrix6x &State::getFrameJacobian(
    const pinocchio::FrameIndex &index,
    const pinocchio::ReferenceFrame &reference_frame) const {
    pinocchio::computeFrameJacobian(model_, *data_, q_, index, reference_frame,
                                    jacobian_);
    return jacobian_;
}

const Eigen::Vector3<Real> &State::getCentreOfMass() const {
    return pinocchio::centerOfMass(model_, *data_, false);
}

Matrix3x State::computeCentreOfMassJacobian() const {
    return pinocchio::jacobianCenterOfMass(model_, *data_, false);
}

}  // namespace osc
