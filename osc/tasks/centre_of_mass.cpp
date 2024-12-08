#include "osc/tasks/centre_of_mass.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>

namespace osc {

void CentreOfMassTask::compute_jacobian(const model_t &model, data_t &data,
                                        const vector_t &q) {
  jacobian_ = pinocchio::jacobianCenterOfMass(model, data);

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
  // e_ = data.oMf[reference_frame_id()].actInv(data.com[0]) - target.position;
  // // Also compute centre of mass velocity
  // e_dot_ =
  //     data.oMf[reference_frame_id()].actInv(data.vcom[0]) - target.velocity;
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
