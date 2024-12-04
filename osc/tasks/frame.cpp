#include "osc/tasks/frame.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

FrameTask::FrameTask(const model_t &model, const std::string &frame_name,
                     const Type &type, const std::string &reference_frame)
    : AbstractTask(),
      type(type),
      frame_name(frame_name),
      reference_frame(reference_frame) {
  // Ensure the model has the provided frames
  if (model.getFrameId(frame_name) == model.frames.size()) {
    assert("Model does not have specified frame");
  }
  if (model.getFrameId(reference_frame) == model.frames.size()) {
    assert("Model does not have specified reference frame");
  }
  if (type == Type::Position) {
    dimension = 3;
  } else if (type == Type::Orientation) {
    dimension = 3;
  } else if (type == Type::Full) {
    dimension = 6;
  }
}

std::shared_ptr<FrameTask> FrameTask::create(
    const model_t &model, const std::string &frame_name, const Type &type,
    const std::string &reference_frame) {
  return std::make_shared<FrameTask>(model, frame_name, type, reference_frame);
}

void FrameTask::jacobian(const model_t &model, data_t &data, const vector_t &q,
                         matrix_t &J) const {
  jacobian_tpl<double>(model, data, q, J);
}

void FrameTask::bias_acceleration(const model_t &model, data_t &data,
                                  const vector_t &q, const vector_t &v,
                                  vector_t &bias) const {
  bias_acceleration_tpl<double>(model, data, q, v, bias);
}

void FrameTask::jacobian(const model_sym_t &model, data_sym_t &data,
                         const vector_sym_t &q, matrix_sym_t &J) const {
  jacobian_tpl<sym_t>(model, data, q, J);
}

void FrameTask::bias_acceleration(const model_sym_t &model, data_sym_t &data,
                                  const vector_sym_t &q, const vector_sym_t &v,
                                  vector_sym_t &bias) const {
  bias_acceleration_tpl<sym_t>(model, data, q, v, bias);
}

void FrameTask::evaluate_error(const model_t &model, data_t &data, vector_t &e,
                               vector_t &e_dot) const {
  VLOG(10) << "FrameTask::evaluate_error()";
  typedef pinocchio::SE3 se3_t;
  typedef pinocchio::Motion twist_t;

  // Frame wrt world
  const se3_t &oMf = data.oMf[model.getFrameId(frame_name)];
  // Reference wrt world
  const se3_t &oMr = data.oMf[model.getFrameId(reference_frame)];
  // Target wrt world
  se3_t oMt = oMr.act(target.pose);
  // Target wrt frame
  se3_t fMt = oMf.actInv(oMt);

  VLOG(10) << "oMf = " << oMf;
  VLOG(10) << "oMr = " << oMr;
  VLOG(10) << "oMt = " << oMt;
  VLOG(10) << "fMt = " << fMt;

  // Compute the error of the system in the local frame
  if (type == Type::Position) {
    e = pinocchio::log6(fMt).linear();
  } else if (type == Type::Orientation) {
    e = pinocchio::log6(fMt).angular();
  } else if (type == Type::Full) {
    e = pinocchio::log6(fMt).toVector();
  }

  // Compute the rate of change of frame
  auto tMf = oMt.actInv(oMf);

  // Construct jacobian of the logarithm map
  // Eigen::Matrix<double, 6, 6> Jlog;
  // pinocchio::Jlog6(tMf, Jlog);

  // Todo - map the error in the velocity into the frame?

  twist_t v =
      pinocchio::getFrameVelocity(model, data, model.getFrameId(frame_name));

  if (type == Type::Position) {
    e_dot = v.linear();
  } else if (type == Type::Orientation) {
    e_dot = v.angular();
  } else if (type == Type::Full) {
    e_dot = v.toVector();
  }
}
}  // namespace osc