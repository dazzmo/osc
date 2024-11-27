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
#include "osc/pid.hpp"
#include "osc/program.hpp"

namespace osc {

// Forward declaration of OSC
class OSC;

/**
 * @brief Parameters specific to a given task
 *
 */
template <typename Vector>
struct task_parameters {
  // Task weighting vector
  Vector w;
  // Desired task acceleration
  Vector xacc_d;
  // todo - Additional parameters that can be added
};

class AbstractCost {};

/**
 * @brief A task of the form Ax + b = J \ddot q + \dot J \dot q and error
 * measure.
 *
 */
class AbstractTask {
 public:
  index_t priority;
  index_t dimension;

  /**
   * @brief Numerical evaluation of a task acceleration of the form J \ddot q +
   * \dot J \dot q
   *
   * @param model
   * @param data
   * @param J The Jacobian of the task (i.e. \grad J \grad q)
   * @param bias The product \dot J \dot q, referred to as the acceleration bias
   * of the task.
   *
   * @note The method assumes that model and data are evaluated to a given
   * state, it does not perform any forward kinematics or dynamics.
   */
  virtual void evaluate(const model_t &model, data_t &data, matrix_t &J,
                        vector_t &bias) = 0;

  // Symbolic evaluation
  virtual void symbolic_evaluate(const model_sym_t &model, data_sym_t &data,
                                 matrix_sym_t &J, vector_sym_t &bias) {
    throw std::runtime_error("symbolic_evaluate not implemented");
  };

  virtual void evaluate_error(const model_t &model, data_t &data, vector_t &e,
                              vector_t &e_dot) = 0;

  struct parameters {
    vector_t weighting;
    vector_t desired;
  };

  parameters parameters;

 private:
};

class FrameTaskNew : public AbstractTask {
 public:
  enum class Type { Position = 0, Orientation, Full };

  FrameTaskNew(const model_t &model, const std::string &frame_name,
               const Type &type = Type::Full,
               const std::string &reference_frame = "universe")
      : AbstractTask(),
        frame_name(frame_name),
        type(type),
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

  static std::shared_ptr<FrameTaskNew> create(
      const model_t &model, const std::string &frame_name,
      const Type &type = Type::Full,
      const std::string &reference_frame = "universe") {
    return std::make_shared<FrameTaskNew>(model, frame_name, type,
                                          reference_frame);
  }

  /**
   * @brief
   *
   * \copydoc osc::AbstractTask::evaluate()
   */
  void evaluate(const model_t &model, data_t &data, matrix_t &J,
                vector_t &bias) override {
    evaluate_tpl<double>(model, data, J, bias);
  }

  void symbolic_evaluate(const model_sym_t &model, data_sym_t &data,
                         matrix_sym_t &J, vector_sym_t &bias) override {
    evaluate_tpl<::casadi::SX>(model, data, J, bias);
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
  void evaluate_tpl(const pinocchio::ModelTpl<T> &model,
                    pinocchio::DataTpl<T> &data, Eigen::MatrixX<T> &J,
                    Eigen::VectorX<T> &dJdq) {
    typename pinocchio::DataTpl<T>::Matrix6x Jfull =
        pinocchio::DataTpl<T>::Matrix6x::Zero(6, model.nv);

    pinocchio::getFrameJacobian(model, data, model.getFrameId(frame_name),
                                pinocchio::LOCAL, Jfull);

    pinocchio::MotionTpl<T> acc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame_name));

    if (type == Type::Position) {
      J = Jfull.topRows(3);
      dJdq = acc.linear();
    } else if (type == Type::Orientation) {
      J = Jfull.bottomRows(3);
      dJdq = acc.angular();
    } else if (type == Type::Full) {
      J = Jfull;
      dJdq = acc.toVector();
    }
  }
};

// class CentreOfMassTaskNew : public AbstractTask {
//  public:
//   struct reference {
//     vector3_t position;
//     vector3_t velocity;
//   };

//   void evaluate(const model_t &model, const data_t &data, matrix_t &J,
//                 vector_t &dJdq) override {
//     evaluate_tpl(model, data, J, dJdq);
//   }

//   virtual void symbolic_evaluate(const model_sym_t &model,
//                                  const data_sym_t &data, sym_t &A, sym_t
//                                  &b)
//                                  {
//     evaluate_tpl(model, data, J, dJdq);
//   };

//   reference target;

//   void evaluate_error(const model_t &model, const data_t &data, vector_t
//   &e,
//                       vector_t &e_dot) {
//     // Compute centre of mass with respect to reference frame
//     e = data.oMf[model.getFrameId(reference_frame)].actInv(data.com[0]) -
//         target.position;
//     // Also compute centre of mass velocity
//     e_dot =
//     data.oMf[model.getFrameId(reference_frame)].actInv(data.vcom[0])
//     -
//             target.velocity;
//   }

//  private:
//   template <typename T>
//   void evaluate_tpl(const pinocchio::ModelTpl<T> &model,
//                     const pinocchio::DataTpl<T> &data, Eigen::MatrixX<T>
//                     &J, Eigen::VectorX<T> &dJdq) {
//     pinocchio::jacobianCenterOfMass(model, data, J);
//     dJdq = data.acom[0];
//   }
// };

/**
 * @brief
 *
 */
template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class TaskTpl : public OSCComponent {
  friend class OSC;

 public:
  // Typedefs
  typedef ValueType value_type;
  typedef IndexType index_type;
  typedef IntegerType integer_type;

  TaskTpl() : dimension_(index_type(0)), pid(index_type(0)) {}

  TaskTpl(const index_type &dim) : dimension_(dim), pid(dim) {
    set_dimension(dim);
  }

  index_type dimension() const { return dimension_; }

  /**
   * @brief Priority of the task, lower values indicate a higher priority
   *
   */
  index_type priority;

  /**
   * @brief PID gains for task tracking
   *
   */
  pid_gains<value_type> pid;

  /**
   * @brief Add the task to the program
   *
   * @param program
   */
  virtual void add_to_program(OSC &program) const = 0;

  /**
   * @brief Compute the error of the task given the current state of the
   * model.
   *
   * @param model
   * @param data
   * @param e
   */
  virtual void compute_task_error(const model_t &model, const data_t &data,
                                  pid_error<value_type> &e) = 0;

  /**
   * @brief Modifyable task parameters
   *
   * @return task_parameters<value_type>&
   */
  task_parameters<vector_t> &parameters() { return parameters_d; }

 protected:
  /**
   * @brief Set the dimension of the task, such that the dimension is the
   * dimension of the error of the system when considering the task (e.g.
   * typically the dimension of the Lie algebra)
   *
   * @param dimension
   */
  void set_dimension(const index_type &dimension) {
    dimension_ = dimension;
    // Create parameters
    parameters_v.w = bopt::create_variable_vector("w", dimension);
    parameters_d.w = vector_t::Ones(dimension);

    parameters_v.xacc_d = bopt::create_variable_vector("xacc_d", dimension);
    parameters_d.xacc_d = vector_t::Zero(dimension);

    pid.resize(dimension);
    pid.p.setOnes();
    pid.i.setOnes();
    pid.d.setZero();
  }

  // Parameter variables
  task_parameters<std::vector<bopt::variable>> parameters_v;
  task_parameters<vector_t> parameters_d;

 private:
  // Dimension of the task
  index_type dimension_;
};

typedef TaskTpl<double> Task;

/**
 * @brief Frame task, used for tracking a desired frame in SE3. Tasks can be
 * purely positional, rotational or the full SE3.
 *
 */
class FrameTask : public Task {
  friend class OSC;

 public:
  /**
   * @brief Types of frame tasks.
   *
   */
  enum class Type { Position = 0, Orientation, Full };

  typedef pinocchio::SE3 se3_t;
  typedef pinocchio::Motion twist_t;

  FrameTask(const model_t &model, const std::string &frame_name,
            const Type &type = Type::Full,
            const std::string &reference_frame = "universe");

  static std::shared_ptr<FrameTask> create(
      const model_t &model, const std::string &frame_name,
      const Type &type = Type::Full,
      const std::string &reference_frame = "universe") {
    return std::make_shared<FrameTask>(model, frame_name, type,
                                       reference_frame);
  }

  struct task_state {
    se3_t pose = se3_t::Identity();
    twist_t twist = twist_t::Zero();
  };

  typedef task_state task_state_t;
  typedef task_state_t reference_t;

  void compute_task_error(const model_t &model, const data_t &data,
                          pid_error<value_type> &e) override;

  reference_t target;

 protected:
  void add_to_program(OSC &program) const override;

 private:
  Type type;
  string_t frame;
  string_t reference_frame;
};

/**
 * @brief Centre of Mass tracking task.
 *
 */
class CentreOfMassTask : public Task {
  friend class OSC;

 public:
  CentreOfMassTask(const model_t &model, const std::string &reference_frame);

  static std::shared_ptr<CentreOfMassTask> create(
      const model_t &model, const std::string &reference_frame = "universe") {
    return std::make_shared<CentreOfMassTask>(model, reference_frame);
  }

  struct task_state {
    vector3_t position = vector3_t::Zero();
    vector3_t velocity = vector3_t::Zero();
  };

  typedef task_state reference_t;

  void compute_task_error(const model_t &model, const data_t &data,
                          pid_error<value_type> &e) override;

  reference_t reference;

 protected:
  void add_to_program(OSC &program) const override;

 private:
  string_t reference_frame;
};

/**
 * @brief Joint tracking task
 *
 */
class JointTrackingTask : public Task {
  friend class OSC;

 public:
  JointTrackingTask(const model_t &model);

  static std::shared_ptr<JointTrackingTask> create(const model_t &model) {
    return std::make_shared<JointTrackingTask>(model);
  }

  struct task_state {
    vector_t joint_position;
    vector_t joint_velocity;
  };

  typedef task_state task_state_t;
  typedef task_state_t reference_t;

  void compute_task_error(const model_t &model, const data_t &data,
                          pid_error<value_type> &e) override;

  reference_t reference;

 protected:
  void add_to_program(OSC &program) const override;

 private:
};

}  // namespace osc