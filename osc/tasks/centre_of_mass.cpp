#include "osc/tasks/centre_of_mass.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace osc {

CentreOfMassTask::CentreOfMassTask(const model_t &model,
                                   const std::string &reference_frame)
    : MotionTask(model, reference_frame) {
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
  reference_ = TrajectoryReference(3, 3, 3);
}

void CentreOfMassTask::compute_jacobian(const model_t &model, data_t &data,
                                        const vector_t &q) {
  pinocchio::getFrameJacobian(model, data, reference_frame_id(),
                              pinocchio::LOCAL, jacobian_full_);

  jacobian_ =
      pinocchio::jacobianCenterOfMass(model, data) - jacobian_full_.topRows(3);
}

void CentreOfMassTask::compute_jacobian_dot_q_dot(const model_t &model,
                                                  data_t &data,
                                                  const vector_t &q,
                                                  const vector_t &v) {
  jacobian_dot_q_dot_ = data.acom[0];
}

void CentreOfMassTask::compute_error(const model_t &model, data_t &data,
                                     const vector_t &q, const vector_t &v) {
  // Compute centre of mass with respect to reference frame
  e_ = data.oMf[reference_frame_id()].actInv(data.com[0]) - reference_.position;
  // Also compute centre of mass velocity
  e_dot_ =
      data.oMf[reference_frame_id()].actInv(data.vcom[0]) - reference_.velocity;
}

void CentreOfMassTask::compute(const model_t &model, data_t &data,
                               const vector_t &q, const vector_t &v) {
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
  compute_error(model, data, q, v);

  // Set desired task acceleration
  xacc_des_ = Kp_.asDiagonal() * e_ + Kd_.asDiagonal() * e_dot_;
}

}  // namespace osc
