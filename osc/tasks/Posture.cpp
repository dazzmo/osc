#include "osc/tasks/posture.hpp"

namespace osc {

PostureTask::PostureTask(const model_t &model)
    : MotionTask(model), nj_(0), q_indices_({}), v_indices_({}) {
  // Set up jacobian for the hinge and prismatic joints
  for (index_t joint_id = 0; joint_id < model.joints.size(); ++joint_id) {
    if (model.joints[joint_id].nq() == 1) {
      if (model.joints[joint_id].idx_q() > 0 &&
          model.joints[joint_id].idx_v() > 0) {
        nj_++;
        q_indices_.push_back(model.joints[joint_id].idx_q());
        v_indices_.push_back(model.joints[joint_id].idx_v());
      }
    }
  }

  this->resize(dim(), model.nv);

  index_t row = 0;
  // Set joint jacobian entries to 1
  for (index_t i : v_indices_) {
    jacobian_(row++, i) = 1.0;
  }

  reference_ = TrajectoryReference(dim(), dim(), dim());
}

void PostureTask::compute_error(const model_t &model, data_t &data,
                                const vector_t &q, const vector_t &v) {
  e_ = reference_.position - q(q_indices_);
  e_dot_ = reference_.velocity - v(v_indices_);
}

void PostureTask::compute(const model_t &model, data_t &data, const vector_t &q,
                          const vector_t &v) {
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
  compute_error(model, data, q, v);

  xacc_des_ = Kp_.asDiagonal() * e_ + Kd_.asDiagonal() * e_dot_ +
              reference_.acceleration;
}

}  // namespace osc
