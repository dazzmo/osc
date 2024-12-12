#include "osc/tasks/frame.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

FrameTask::FrameTask(const model_t &model, const std::string &frame,
                     const Type &type)
    : MotionTask(model), FrameSE3(model, frame, type) {}

std::shared_ptr<FrameTask> FrameTask::create(const model_t &model,
                                             const std::string &frame,
                                             const Type &type) {
  return std::make_shared<FrameTask>(model, frame, type);
}

void FrameTask::compute_jacobian(const model_t &model, data_t &data,
                                 const vector_t &q) {
  FrameSE3::compute_jacobian(model, data, jacobian_);
}

void FrameTask::compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                           const vector_t &q,
                                           const vector_t &v) {
  FrameSE3::compute_jacobian_dot_q_dot(model, data, jacobian_dot_q_dot_);
}

void FrameTask::compute_error(const model_t &model, data_t &data,
                              const vector_t &q, const vector_t &v) {
  FrameSE3::compute_error(model, data, get_reference().position,
                          get_reference().velocity, e_, e_dot_);
}

void FrameTask::compute(const model_t &model, data_t &data, const vector_t &q,
                        const vector_t &v) {
  // Compute jacobian and derivative
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
  compute_error(model, data, q, v);

  // Compute desired task acceleration
  xacc_des_ =
      Kp_.asDiagonal() * get_error() + Kd_.asDiagonal() * get_error_dot();

  if (get_type() == Type::Position) {
    xacc_des_ += get_reference().acceleration.linear();
  } else if (get_type() == Type::Orientation) {
    xacc_des_ += get_reference().acceleration.angular();
  } else {
    xacc_des_ += get_reference().acceleration.toVector();
  }
}

}  // namespace osc