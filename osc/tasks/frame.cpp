#include "osc/tasks/frame.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

FrameTask::FrameTask(const model_t &model, const std::string &frame,
                     const Type &type, const std::string &reference_frame)
    : MotionTask(model, reference_frame), type_(type), frame_(frame) {
  // Ensure the model has the provided frames
  frame_id_ = model.getFrameId(frame_);
  if (frame_id_ == model.frames.size()) {
    assert("Model does not have specified frame");
  }
  set_type(type);
  // Initialise full jacobian for computations
  jacobian_full_ = pinocchio::Data::Matrix6x::Zero(6, model.nv);
}

void FrameTask::set_type(const Type &type) {
  // Set type
  type_ = type;
  // Error
  e_ = vector_t::Zero(dim());
  e_dot_ = vector_t::Zero(dim());

  // Default gains
  Kp_ = vector_t::Ones(dim());
  Kd_ = vector_t::Ones(dim());

  // Desired task acceleration
  xacc_des_ = vector_t::Zero(dim());
}

std::shared_ptr<FrameTask> FrameTask::create(
    const model_t &model, const std::string &frame, const Type &type,
    const std::string &reference_frame) {
  return std::make_shared<FrameTask>(model, frame, type, reference_frame);
}

void FrameTask::compute_jacobian(const model_t &model, data_t &data,
                                 const vector_t &q) {
  pinocchio::getFrameJacobian(model, data, frame_id(),
                              pinocchio::LOCAL_WORLD_ALIGNED, jacobian_full_);
  if (get_type() == Type::Position) {
    jacobian_ = jacobian_full_.topRows(3);
  } else if (get_type() == Type::Orientation) {
    jacobian_ = jacobian_full_.bottomRows(3);
  } else if (get_type() == Type::Full) {
    jacobian_ = jacobian_full_;
  }
}

void FrameTask::compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                           const vector_t &q,
                                           const vector_t &v) {
  pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
      model, data, frame_id(), pinocchio::LOCAL_WORLD_ALIGNED);

  if (get_type() == Type::Position) {
    jacobian_dot_q_dot_ = acc.linear();
  } else if (get_type() == Type::Orientation) {
    jacobian_dot_q_dot_ = acc.angular();
  } else if (get_type() == Type::Full) {
    jacobian_dot_q_dot_ = acc.toVector();
  }
}

void FrameTask::compute_error(const model_t &model, data_t &data,
                              const vector_t &q, const vector_t &v) {
  typedef pinocchio::SE3 se3_t;
  typedef pinocchio::Motion twist_t;
  // Compute error in pose
  // Frame wrt world
  const se3_t &oMf = data.oMf[frame_id()];
  se3_t fMt = oMf.actInv(get_reference().position);

  // Get error in local world-aligned frame
  se3_t wMl = se3_t::Identity();
  wMl.rotation(oMf.rotation());

  // Compute error in the local frame
  auto p_err_l = wMl.toActionMatrix() * pinocchio::log6(fMt).toVector();

  // Compute Jacobian of point
  twist_t v_err = reference_.velocity -
                  pinocchio::getFrameVelocity(model, data, frame_id(),
                                              pinocchio::LOCAL_WORLD_ALIGNED);

  if (get_type() == Type::Position) {
    e_ = p_err_l.topRows(3);
    e_dot_ = v_err.linear();
  } else if (get_type() == Type::Orientation) {
    e_ = p_err_l.bottomRows(3);
    e_dot_ = v_err.angular();
  } else if (get_type() == Type::Full) {
    e_ = p_err_l;
    e_dot_ = v_err.toVector();
  }
}

void FrameTask::compute(const model_t &model, data_t &data, const vector_t &q,
                        const vector_t &v) {
  // Compute jacobian and derivative
  compute_jacobian(model, data, q);
  compute_jacobian_dot_q_dot(model, data, q, v);
  compute_error(model, data, q, v);

  // Compute desired task acceleration
  xacc_des_ = Kp_.asDiagonal() * e_ + Kd_.asDiagonal() * e_dot_;
  if (get_type() == Type::Position) {
    xacc_des_ += reference_.acceleration.linear();
  } else if (get_type() == Type::Orientation) {
    xacc_des_ += reference_.acceleration.angular();
  } else {
    xacc_des_ += reference_.acceleration.toVector();
  }
}

}  // namespace osc