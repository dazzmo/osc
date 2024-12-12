#include "osc/se3.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

FrameSE3::FrameSE3(const model_t &model, const std::string &frame,
                   const Type &type)
    : type_(type),
      frame_(frame),
      reference_frame_("universe"),
      use_local_frame_(false) {
  // Ensure the model has the provided frames
  frame_id_ = model.getFrameId(this->frame());

  if (frame_id_ == model.frames.size()) {
    assert("Model does not have specified frame");
  }

  // Initialise full jacobian for computations
  jacobian_full_ = pinocchio::Data::Matrix6x::Zero(6, model.nv);
}

index_t FrameSE3::dim() const {
  if (get_type() == Type::Position || get_type() == Type::Orientation) {
    return 3;
  } else {
    return 6;
  }
}

void FrameSE3::set_type(const Type &type) {
  // Set type
  type_ = type;
}

void FrameSE3::compute_jacobian(const model_t &model, data_t &data,
                                matrix_t &jacobian) {
  if (use_local_frame_) {
    pinocchio::getFrameJacobian(model, data, frame_id(), pinocchio::LOCAL,
                                jacobian_full_);
  } else {
    // Use world-aligned frame positioned at specified frame
    pinocchio::getFrameJacobian(model, data, frame_id(),
                                pinocchio::LOCAL_WORLD_ALIGNED, jacobian_full_);
  }

  if (get_type() == Type::Position) {
    jacobian = jacobian_full_.topRows(3);
  } else if (get_type() == Type::Orientation) {
    jacobian = jacobian_full_.bottomRows(3);
  } else if (get_type() == Type::Full) {
    jacobian = jacobian_full_;
  }
}

void FrameSE3::compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                          vector_t &jacobian_dot_q_dot) {
  pinocchio::Motion acc;

  if (use_local_frame_) {
    pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
        model, data, frame_id(), pinocchio::LOCAL);
  } else {
    pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
        model, data, frame_id(), pinocchio::LOCAL_WORLD_ALIGNED);
  }

  if (get_type() == Type::Position) {
    jacobian_dot_q_dot = acc.linear();
  } else if (get_type() == Type::Orientation) {
    jacobian_dot_q_dot = acc.angular();
  } else if (get_type() == Type::Full) {
    jacobian_dot_q_dot = acc.toVector();
  }
}

void FrameSE3::compute_error(const model_t &model, data_t &data,
                             const pinocchio::SE3 &p_ref,
                             const pinocchio::Motion &v_ref, vector_t &e,
                             vector_t &e_dot) {
  typedef pinocchio::SE3 se3_t;
  typedef pinocchio::Motion twist_t;

  pinocchio::Motion p_err, v_err;

  if (use_local_frame_) {
    // Frame wrt world
    const se3_t &oMf = data.oMf[frame_id()];
    // Get error of the pose with respect to the frame
    se3_t fMt = oMf.actInv(p_ref);
    // Compute error in the local frame
    p_err = pinocchio::log6(fMt);
    // Compute Jacobian of point
    v_err = v_ref - pinocchio::getFrameVelocity(model, data, frame_id(),
                                                pinocchio::LOCAL);
  } else {
    // Frame wrt world
    const se3_t &oMf = data.oMf[frame_id()];
    // Get error of the pose with respect to the target frame
    se3_t fMt = oMf.actInv(p_ref);

    // Get error in local world-aligned frame
    se3_t wMl = se3_t::Identity();
    wMl.rotation(oMf.rotation());
    // Compute error in the local frame
    p_err = wMl.act(pinocchio::log6(fMt));
    // Compute Jacobian of point
    v_err = v_ref - pinocchio::getFrameVelocity(model, data, frame_id(),
                                                pinocchio::LOCAL_WORLD_ALIGNED);
  }

  if (get_type() == Type::Position) {
    e = p_err.linear();
    e_dot = v_err.linear();
  } else if (get_type() == Type::Orientation) {
    e = p_err.angular();
    e_dot = v_err.angular();
  } else if (get_type() == Type::Full) {
    e = p_err.toVector();
    e_dot = v_err.toVector();
  }
}

}  // namespace osc