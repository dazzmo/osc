#include "osc/tasks/posture.hpp"

namespace osc {

void PostureTask::compute(const model_t &model, data_t &data, const vector_t &q,
                          const vector_t &v) {
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);

  e_ = q;
  e_dot_ = v;

  xacc_des_ = Kp_.asDiagonal() * e_ + Kd_.asDiagonal() * e_dot_;
}

}  // namespace osc
