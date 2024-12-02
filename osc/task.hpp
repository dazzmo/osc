#pragma once

#include <bopt/program.hpp>

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>
#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/common.hpp"
#include "osc/cost.hpp"
#include "osc/holonomic.hpp"
#include "osc/pid.hpp"

namespace osc {

// Forward declaration of OSC
class OSC;

class AbstractTask : public HolonomicExpression {
 public:
  /**
   * @brief Priority of the task, with lower values indicating higher priority.
   *
   */
  index_t priority;

  virtual void evaluate_error(const model_t &model, data_t &data, vector_t &e,
                              vector_t &e_dot) = 0;

 private:
};

class FrameTask : public AbstractTask {
 public:
  enum class Type { Position = 0, Orientation, Full };

  FrameTask(const model_t &model, const std::string &frame_name,
            const Type &type = Type::Full,
            const std::string &reference_frame = "universe")
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

  static std::shared_ptr<FrameTask> create(
      const model_t &model, const std::string &frame_name,
      const Type &type = Type::Full,
      const std::string &reference_frame = "universe") {
    return std::make_shared<FrameTask>(model, frame_name, type,
                                       reference_frame);
  }

  void jacobian(const model_t &model, data_t &data, matrix_t &J) override {
    jacobian_tpl<double>(model, data, J);
  }

  void bias_acceleration(const model_t &model, data_t &data,
                         vector_t &bias) override {
    bias_acceleration_tpl<double>(model, data, bias);
  }

  void jacobian(const model_sym_t &model, data_sym_t &data,
                matrix_sym_t &J) override {
    jacobian_tpl<sym_t>(model, data, J);
  }

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         vector_sym_t &bias) override {
    bias_acceleration_tpl<sym_t>(model, data, bias);
  }

  Type type;

  string_t frame_name;
  string_t reference_frame;

  struct reference {
    pinocchio::SE3 pose;
    pinocchio::Motion twist;
  };

  reference target;

  void evaluate_error(const model_t &model, data_t &data, vector_t &e,
                      vector_t &e_dot) {
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

 private:
  template <typename T>
  void jacobian_tpl(const pinocchio::ModelTpl<T> &model,
                    pinocchio::DataTpl<T> &data, Eigen::MatrixX<T> &J) {
    typename pinocchio::DataTpl<T>::Matrix6x Jfull =
        pinocchio::DataTpl<T>::Matrix6x::Zero(6, model.nv);

    pinocchio::getFrameJacobian(model, data, model.getFrameId(frame_name),
                                pinocchio::LOCAL, Jfull);
    if (type == Type::Position) {
      J = Jfull.topRows(3);
    } else if (type == Type::Orientation) {
      J = Jfull.bottomRows(3);
    } else if (type == Type::Full) {
      J = Jfull;
    }
  }

  template <typename T>
  void bias_acceleration_tpl(const pinocchio::ModelTpl<T> &model,
                             pinocchio::DataTpl<T> &data,
                             Eigen::VectorX<T> &bias) {
    pinocchio::MotionTpl<T> acc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame_name));

    if (type == Type::Position) {
      bias = acc.linear();
    } else if (type == Type::Orientation) {
      bias = acc.angular();
    } else if (type == Type::Full) {
      bias = acc.toVector();
    }
  }
};

class CentreOfMassTask : public AbstractTask {
 public:
  CentreOfMassTask(const model_t &model,
                      const std::string &reference_frame = "universe")
      : AbstractTask() {}

  struct reference {
    vector3_t position;
    vector3_t velocity;
  };

  void jacobian(const model_t &model, data_t &data, matrix_t &J) override {
    jacobian_tpl<double>(model, data, J);
  }

  void bias_acceleration(const model_t &model, data_t &data,
                         vector_t &bias) override {
    bias_acceleration_tpl<double>(model, data, bias);
  }

  void jacobian(const model_sym_t &model, data_sym_t &data,
                matrix_sym_t &J) override {
    jacobian_tpl<sym_t>(model, data, J);
  }

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         vector_sym_t &bias) override {
    bias_acceleration_tpl<sym_t>(model, data, bias);
  }

  reference target;
  string_t reference_frame;

  void evaluate_error(const model_t &model, const data_t &data, vector_t &e,
                      vector_t &e_dot) {
    // Compute centre of mass with respect to reference frame
    e = data.oMf[model.getFrameId(reference_frame)].actInv(data.com[0]) -
        target.position;
    // Also compute centre of mass velocity
    e_dot = data.oMf[model.getFrameId(reference_frame)].actInv(data.vcom[0]) -
            target.velocity;
  }

 private:
  template <typename T>
  void evaluate_tpl(const pinocchio::ModelTpl<T> &model,
                    const pinocchio::DataTpl<T> &data, Eigen::MatrixX<T> &J,
                    Eigen::VectorX<T> &dJdq) {
    pinocchio::jacobianCenterOfMass(model, data, J);
    dJdq = data.acom[0];
  }

 private:
  template <typename T>
  void jacobian_tpl(const pinocchio::ModelTpl<T> &model,
                    pinocchio::DataTpl<T> &data, Eigen::MatrixX<T> &J) {
    J = pinocchio::jacobianCenterOfMass(model, data);
  }

  template <typename T>
  void bias_acceleration_tpl(const pinocchio::ModelTpl<T> &model,
                             pinocchio::DataTpl<T> &data,
                             Eigen::VectorX<T> &bias) {
    bias = data.acom[0];
  }
};

class JointTrackingTask : public AbstractTask {
 public:
  JointTrackingTask(const model_t &model) : AbstractTask() {}

  struct reference {
    vector_t position;
    vector_t velocity;
  };

  void jacobian(const model_t &model, data_t &data, matrix_t &J) override {
    // For all joints in the system, set the entries to 1
  }

  void bias_acceleration(const model_t &model, data_t &data,
                         vector_t &bias) override {
    bias.setZero();
  }

  void jacobian(const model_sym_t &model, data_sym_t &data,
                matrix_sym_t &J) override {}

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         vector_sym_t &bias) override {
    bias.setZero();
  }

  reference target;
  string_t reference_frame;

  void evaluate_error(const model_t &model, const data_t &data, vector_t &e,
                      vector_t &e_dot) {
    // Compute centre of mass with respect to reference frame
    e.setZero();
    e_dot.setZero();
  }

 private:
};

class AbstractTaskCost : public AbstractQuadraticCost {
 public:
  AbstractTaskCost(const model_t &model,
                   const std::shared_ptr<AbstractTask> &task)
      : AbstractQuadraticCost() {}

  /**
   * @brief The task the cost is associated with
   *
   * @return std::shared_ptr<AbstractTask>&
   */
  std::shared_ptr<AbstractTask> &task() { return task_; }

 protected:
  std::shared_ptr<AbstractTask> task_;
};

/**
 * @brief Weighted task cost of the form \f$ || \ddot x - \ddot x_d ||_w^2 \f$
 *
 */
class WeightedTaskCost : public AbstractTaskCost {
 public:
  WeightedTaskCost(const model_t &model,
                   const std::shared_ptr<AbstractTask> &task)
      : AbstractTaskCost(model, task), pid(task->dimension) {
    parameters.w = bopt::create_variable_vector("w", task->dimension);
    parameters.x = bopt::create_variable_vector("x", task->dimension);
  }

  struct parameters {
    std::vector<bopt::variable> w;
    std::vector<bopt::variable> x;
  };

  parameters parameters;

  // PID tracking gains
  pid_gains<double> pid;

  // Weighting
  vector_t w;
  // Desired task second derivative
  vector_t x;

  bopt::quadratic_cost<double>::shared_ptr to_cost(const model_t &model) const {
    LOG(INFO) << "to_cost";

    // Compute the target frame in the contact frame of the model
    vector_sym_t q = create_symbolic_vector("q", model.nq);
    vector_sym_t v = create_symbolic_vector("v", model.nv);
    vector_sym_t a = create_symbolic_vector("a", model.nv);
    vector_sym_t e = create_symbolic_vector("e", task_->dimension);

    // Compute frame state
    model_sym_t model_sym = model.cast<sym_t>();
    data_sym_t data_sym(model_sym);

    // Compute the kinematic tree state of the system
    pinocchio::forwardKinematics(model_sym, data_sym, q, v, a);
    pinocchio::updateFramePlacements(model_sym, data_sym);

    // Compute Jacobian and bias
    matrix_sym_t J(task_->dimension, model.nv);
    vector_sym_t bias(task_->dimension);

    task_->jacobian(model_sym, data_sym, J);
    task_->bias_acceleration(model_sym, data_sym, bias);

    // Set difference between task second derivative and desired
    vector_sym_t error = J * a - bias - e;

    vector_sym_t w = create_symbolic_vector("w", task_->dimension);

    sym_t cost = error.transpose() * w.asDiagonal() * error;

    return bopt::casadi::quadratic_cost<double>::create(
        cost, casadi::eigen_to_casadi(a),
        sym_vector_t({casadi::eigen_to_casadi(q), casadi::eigen_to_casadi(v),
                      casadi::eigen_to_casadi(w), casadi::eigen_to_casadi(e)}));
  }
};

class HeirarchicalTaskCost {
  // todo - custom qudratic cost
  // class Cost : public bopt::quadratic_cost<double> {};
};

}  // namespace osc