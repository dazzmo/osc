#pragma once

#include <bopt/program.hpp>

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>
#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>

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