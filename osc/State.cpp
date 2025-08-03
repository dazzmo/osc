#include "osc/State.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace osc {

State::State(const Model &model)
    : model_(model),
      data_(std::make_unique<ModelData>(model)),
      q_(pinocchio::neutral(model)),
      v_(Vector::Zero(model.nv)),
      jacobian_(Matrix6x::Zero(6, model_.nv)) {}

void State::update(const ConfigVectorType &q, const TangentVectorType &v) {
    q_ = q;
    v_ = v;

    pinocchio::forwardKinematics(model_, *data_, q, v);
    pinocchio::updateFramePlacements(model_, *data_);
    pinocchio::computeJointJacobians(model_, *data_);
    pinocchio::centerOfMass(model_, *data_);
    pinocchio::crba(model_, *data_, q);
}

pinocchio::JointIndex State::getJointIndex(const String &joint) const {
    const auto index = model_.getJointId(joint);
    assert(index < model_.joints.size() && "Joint does not exist");
    return index;
}
pinocchio::FrameIndex State::getFrameIndex(const String &frame) const {
    const auto index = model_.getFrameId(frame);
    assert(index < model_.frames.size() && "Frame does not exist");
    return index;
}

SE3 State::getTransformFrameToWorld(const String &frame) const {
    return data_->oMf[getFrameIndex(frame)];
}

const State::Matrix6x &State::computeJointJacobian(
    const pinocchio::JointIndex &index) const {
    pinocchio::computeJointJacobian(model_, *data_, q_, index, jacobian_);
    return jacobian_;
}

const State::Matrix6x &State::computeFrameJacobian(
    const pinocchio::FrameIndex &index,
    const pinocchio::ReferenceFrame &reference_frame) const {
    pinocchio::computeFrameJacobian(model_, *data_, q_, index, reference_frame,
                                    jacobian_);
    return jacobian_;
}

 Motion State::getFrameVelocity(
    const pinocchio::FrameIndex &index,
    const pinocchio::ReferenceFrame &reference_frame) const {
    return pinocchio::getFrameVelocity(model_, *data_, index, reference_frame);
}

 Motion State::getFrameClassicalAcceleration(
    const pinocchio::FrameIndex &index,
    const pinocchio::ReferenceFrame &reference_frame) const {
    return pinocchio::getFrameClassicalAcceleration(model_, *data_, index,
                                                    reference_frame);
}

const Vector3 &State::getCentreOfMass() const {
    return data_->com[0];
}

const Vector3 &State::getCentreOfMassVelocity() const {
    return data_->vcom[0];
}

const Vector3 &State::getCentreOfMassAcceleration() const {
    return data_->acom[0];
}

const State::Matrix3x &State::computeCentreOfMassJacobian() const {
    return pinocchio::jacobianCenterOfMass(model_, *data_, false);
}

const Matrix &State::getInertiaMatrix() const { return data_->M; }

Vector State::computeNonlinearEffects(const Eigen::Ref<const Vector> &q,
                                      const Eigen::Ref<const Vector> &v) const {
    return pinocchio::nonLinearEffects(model_, *data_, q, v);
}

}  // namespace osc
