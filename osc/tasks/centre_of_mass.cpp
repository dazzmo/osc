#include "osc/tasks/centre_of_mass.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace osc {

CentreOfMassTask::CentreOfMassTask(const model_t &model) : MotionTask(model) {
  jacobian_full_ = matrix_t::Zero(6, model.nv);
  // Error
  e_ = vector_t::Zero(dim());
  e_dot_ = vector_t::Zero(dim());

  // Default gains
  Kp_ = vector_t::Ones(dim());
  Kd_ = vector_t::Ones(dim());

  // Desired task acceleration
  xacc_des_ = vector_t::Zero(dim());

  // Set reference dimension
  reference_ = TrajectoryReference(dim(), dim(), dim());
}

void CentreOfMassTask::compute_jacobian(const model_t &model, data_t &data,
                                        const vector_t &q) {
  // // Compute in the required reference frame
  // auto &oMr = data.oMf[reference_frame_id()];
  // // Get Jacobian of reference frame with respect to world
  // pinocchio::getFrameJacobian(model, data, reference_frame_id(),
  //                             pinocchio::WORLD, jacobian_full_);

  jacobian_ = pinocchio::jacobianCenterOfMass(model, data, false);
}

void CentreOfMassTask::compute_jacobian_dot_q_dot(const model_t &model,
                                                  data_t &data,
                                                  const vector_t &q,
                                                  const vector_t &v) {
  // Compute in the required reference frame
  // auto &oMr = data.oMf[reference_frame_id()];

  // pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
  //     model, data, reference_frame_id(), pinocchio::WORLD);

  // Get reference frame point acceleration
  jacobian_dot_q_dot_ = data.acom[0];
  // oMr.actInv(data.acom[0] - acc.linear());
}

void CentreOfMassTask::compute_error(const model_t &model, data_t &data,
                                     const vector_t &q, const vector_t &v) {
  // Compute centre of mass with respect to reference frame
  e_ = get_reference().position -
       data.oMf[reference_frame_id()].actInv(data.com[0]);
  // Also compute centre of mass velocity
  e_dot_ = get_reference().velocity -
           data.oMf[reference_frame_id()].actInv(data.vcom[0]);
}

void CentreOfMassTask::compute(const model_t &model, data_t &data,
                               const vector_t &q, const vector_t &v) {
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
  compute_error(model, data, q, v);

  // Set desired task acceleration
  xacc_des_ = Kp_.asDiagonal() * e_ + Kd_.asDiagonal() * e_dot_ +
              reference_.acceleration;
}

}  // namespace osc
