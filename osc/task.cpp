#include "osc/task.hpp"

#include <bopt/ad/casadi/casadi.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/osc.hpp"

namespace osc {

FrameTask::FrameTask(const model_t &model, const std::string &frame,
                     const Type &type, const std::string &reference_frame)
    : Task(), frame(frame), type(type), reference_frame(reference_frame) {
  // Ensure the model has the provided frames
  if (model.getFrameId(frame) == model.frames.size()) {
    assert("Model does not have specified frame");
  }
  if (model.getFrameId(reference_frame) == model.frames.size()) {
    assert("Model does not have specified reference frame");
  }
  if (type == Type::Position) {
    set_dimension(3);
  } else if (type == Type::Orientation) {
    set_dimension(3);
  } else if (type == Type::Full) {
    set_dimension(6);
  }
}

void FrameTask::add_to_program(OSC &osc_program) const {
  const model_sym_t &model = OSCComponent::get_model(osc_program);
  bopt::mathematical_program<double> &program =
      OSCComponent::get_program(osc_program);
  const osc_variables &variables =
      OSCComponent::get_variables(osc_program);
  osc_parameters &parameters =
      OSCComponent::get_parameters(osc_program);

  // Add to program
  vector_sym_t q = create_symbolic_vector("q", model.nq);
  vector_sym_t v = create_symbolic_vector("v", model.nv);
  vector_sym_t a = create_symbolic_vector("a", model.nv);

  vector_sym_t w = create_symbolic_vector("w", dimension());
  vector_sym_t xacc_d = create_symbolic_vector("xacc_d", dimension());

  pinocchio::DataTpl<sym_t> data(model);

  // Compute the kinematic tree state of the system
  pinocchio::forwardKinematics(model, data, q, v, a);
  pinocchio::updateFramePlacements(model, data);

  // Compute the acceleration of the target frame in its LOCAL frame
  pinocchio::MotionTpl<sym_t> acc = pinocchio::getFrameClassicalAcceleration(
      model, data, model.getFrameId(frame));

  // Determine error in frame acceleration (in LOCAL frame)
  vector_sym_t e;
  if (type == Type::Position) {
    e = acc.linear() - xacc_d;
  } else if (type == Type::Orientation) {
    e = acc.angular() - xacc_d;
  } else if (type == Type::Full) {
    e = acc.toVector() - xacc_d;
  }

  sym_t q_s = casadi::eigen_to_casadi(q);
  sym_t v_s = casadi::eigen_to_casadi(v);
  sym_t a_s = casadi::eigen_to_casadi(a);
  sym_t w_s = casadi::eigen_to_casadi(w);
  sym_t xacc_d_s = casadi::eigen_to_casadi(xacc_d);

  // Compute weighted squared norm
  sym_t cost = e.transpose() * w.asDiagonal() * e;

  // Register parameters with the program
  program.add_parameters(parameters_v.w);
  program.add_parameters(parameters_v.xacc_d);

  program.add_quadratic_cost(
      bopt::casadi::quadratic_cost<value_type>::create(
          cost, a_s, sym_vector_t({q_s, v_s, w_s, xacc_d_s})),
      // Variables
      {variables.a},
      // Parameters
      {parameters.q, parameters.v,
       // Task-specific parameters
       parameters_v.w, parameters_v.xacc_d});
};

void FrameTask::compute_task_error(const model_t &model, const data_t &data,
                                   pid_error<double> &e) {
  // Frame wrt world
  const se3_t &oMf = data.oMf[model.getFrameId(frame)];
  // Reference wrt world
  const se3_t &oMr = data.oMf[model.getFrameId(reference_frame)];
  // Target wrt world
  se3_t oMt = oMr.act(target.pose);
  // Target wrt frame
  se3_t fMt = oMf.actInv(oMt);

  // Compute the error of the system in the local frame
  if (type == Type::Position) {
    e.error = pinocchio::log6(fMt).linear();
  } else if (type == Type::Orientation) {
    e.error = pinocchio::log6(fMt).angular();
  } else if (type == Type::Full) {
    e.error = pinocchio::log6(fMt).toVector();
  }

  // Compute the rate of change of frame
  auto tMf = oMt.actInv(oMf);

  // Construct jacobian of the logarithm map
  // Eigen::Matrix<double, 6, 6> Jlog;
  // pinocchio::Jlog6(tMf, Jlog);

  // Todo - map the error in the velocity into the frame?

  twist_t v = pinocchio::getFrameVelocity(model, data, model.getFrameId(frame));

  if (type == Type::Position) {
    e.error_dot = v.linear();
  } else if (type == Type::Orientation) {
    e.error_dot = v.angular();
  } else if (type == Type::Full) {
    e.error_dot = v.toVector();
  }
}

CentreOfMassTask::CentreOfMassTask(const model_t &model,
                                   const std::string &reference_frame)
    : Task(3), reference_frame(reference_frame) {}

void CentreOfMassTask::add_to_program(OSC &osc_program) const {
  const model_sym_t &model = OSCComponent::get_model(osc_program);
  bopt::mathematical_program<double> &program =
      OSCComponent::get_program(osc_program);
  const osc_variables &variables =
      OSCComponent::get_variables(osc_program);
  osc_parameters &parameters =
      OSCComponent::get_parameters(osc_program);

  vector_sym_t q = create_symbolic_vector("q", model.nq);
  vector_sym_t v = create_symbolic_vector("v", model.nv);
  vector_sym_t a = create_symbolic_vector("a", model.nv);

  vector_sym_t w = create_symbolic_vector("w", dimension());
  vector_sym_t xacc_d = create_symbolic_vector("xacc_d", dimension());

  pinocchio::DataTpl<sym_t> data(model);

  // Compute the kinematic tree state of the system
  pinocchio::forwardKinematics(model, data, q, v, a);
  pinocchio::centerOfMass(model, data, false);
  pinocchio::updateFramePlacements(model, data);

  // Compute the acceleration of the target frame in its LOCAL frame
  vector_sym_t com_acc = data.acom[0];

  // Determine error in frame acceleration (in LOCAL frame)
  vector_sym_t e = com_acc - xacc_d;

  sym_t q_s = casadi::eigen_to_casadi(q);
  sym_t v_s = casadi::eigen_to_casadi(v);
  sym_t a_s = casadi::eigen_to_casadi(a);
  sym_t w_s = casadi::eigen_to_casadi(w);
  sym_t xacc_d_s = casadi::eigen_to_casadi(xacc_d);

  // Compute weighted squared norm
  sym_t cost = e.transpose() * w.asDiagonal() * e;

  // Register parameters with the program
  program.add_parameters(parameters_v.w);
  program.add_parameters(parameters_v.xacc_d);

  program.add_quadratic_cost(
      bopt::casadi::quadratic_cost<value_type>::create(
          cost, a_s, sym_vector_t({q_s, v_s, w_s, xacc_d_s})),
      // Variables
      {variables.a},
      // Parameters
      {parameters.q, parameters.v,
       // Task-specific parameters
       parameters_v.w, parameters_v.xacc_d});
}

void CentreOfMassTask::compute_task_error(const model_t &model,
                                          const data_t &data,
                                          pid_error<double> &e) {
  // Compute centre of mass with respect to reference frame
  e.error = data.oMf[model.getFrameId(reference_frame)].actInv(data.com[0]) -
            reference.position;
  // Also compute centre of mass velocity
  e.error_dot =
      data.oMf[model.getFrameId(reference_frame)].actInv(data.vcom[0]) -
      reference.velocity;
}

JointTrackingTask::JointTrackingTask(const model_t &model) : Task(model.nq) {}

void JointTrackingTask::add_to_program(OSC &osc_program) const {
  const model_sym_t &model = OSCComponent::get_model(osc_program);
  bopt::mathematical_program<double> &program =
      OSCComponent::get_program(osc_program);
  const osc_variables &variables =
      OSCComponent::get_variables(osc_program);
  osc_parameters &parameters =
      OSCComponent::get_parameters(osc_program);

  vector_sym_t a = create_symbolic_vector("a", model.nv);
  vector_sym_t w = create_symbolic_vector("w", dimension());
  vector_sym_t xacc_d = create_symbolic_vector("xacc_d", dimension());

  // Determine error as measure of joint accelerations
  vector_sym_t e = a - xacc_d;

  sym_t a_s = casadi::eigen_to_casadi(a);
  sym_t w_s = casadi::eigen_to_casadi(w);
  sym_t xacc_d_s = casadi::eigen_to_casadi(xacc_d);

  // Compute weighted squared norm
  sym_t cost = e.transpose() * w.asDiagonal() * e;

  // Register parameters with the program
  program.add_parameters(parameters_v.w);
  program.add_parameters(parameters_v.xacc_d);

  program.add_quadratic_cost(bopt::casadi::quadratic_cost<value_type>::create(
                                 cost, a_s, sym_vector_t({w_s, xacc_d_s})),
                             // Variables
                             {variables.a},
                             // Parameters
                             {parameters.q, parameters.v,
                              // Task-specific parameters
                              parameters_v.w, parameters_v.xacc_d});
}

void JointTrackingTask::compute_task_error(const model_t &model,
                                           const data_t &data,
                                           pid_error<double> &e) {
  // Compute centre of mass with respect to reference frame
  e.error = reference.joint_position;
  // Also compute centre of mass velocity
  e.error_dot = reference.joint_velocity;
}

}  // namespace osc