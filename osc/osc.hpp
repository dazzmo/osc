/**
 * @file osc.hpp
 * @author your name (you@domain.com)
 * @brief Weighted Operational Space Control
 * @version 0.1
 * @date 2024-11-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <bopt/program.hpp>
#include <bopt/solvers/qpoases.hpp>

#include "osc/constraint.hpp"
#include "osc/contact.hpp"
#include "osc/cost.hpp"
#include "osc/dynamics.hpp"
#include "osc/task.hpp"

namespace osc {

class OSC {
  friend class OSCComponent;

 public:
  OSC(model_t &model, const index_t &nu)
      : model(model), model_sym(model.cast<sym_t>()), program_() {
    // Create system dynamics
    variables_.a = bopt::create_variable_vector("a", model.nv);
    variables_.u = bopt::create_variable_vector("u", nu);
    parameters_.q = bopt::create_variable_vector("q", model.nq);
    parameters_.v = bopt::create_variable_vector("v", model.nv);

    // Add variables to the program
    program_.add_variables(variables_.a);
    program_.add_parameters(parameters_.q);
    program_.add_parameters(parameters_.v);
  }

  /**
   * @brief Task cost || A x - b || ^2_W which evaluates to || J \ddot q + \dot
   * J \dot q - \ddot x_d ||_W^2
   *
   */
  class TaskCost : public bopt::quadratic_cost<double> {
   public:
    // Evaluate it numerically?
    enum class Method { Numeric, Symbolic, Codegen };

    TaskCost(const std::shared_ptr<AbstractTask> &task) {
      // If codegen
      // casadi::SX J = task->evaluate(model, data, J, dJdq);
      // casadi::SX cost = mtimes(J.)

      // Create expression or evaluate numerically
    }

    integer_type A(const double **arg, double *res) override {
      // Take arguements
      // Eigen::Map<const vector_t> w(arg[0]);
      // Eigen::Map<vector_t> xacc(arg[1]);

      // if (type == Numeric) {
      //   Eigen::Map<matrix_t> A(res[0]);
      //   A = J_.transpose() * w.asDiagonal() * J_;
      // }

      // if (type == Symbolic) {
      // }

      // if (type == Codegen) {
      // }

      return 0;
    }

    integer_type A_info(out_info_t &info) {
      // info.m = 10;
      // info.n = 10;
      // info.sparsity = bopt::matrix_type::Dense;
      return 0;
    }

    integer_type b(const double **arg, double *res) override {
      // Take arguements
      // Eigen::Map<vector_t> w(arg[0]);
      // Eigen::Map<vector_t> xacc(arg[1]);
      // Eigen::Map<matrix_t> b(res[0]);

      // b = dJdq_ - xaccd;

      // if (codegen) {
      //   // Evaluate as normal
      //   // b_cg(arg, res);
      // }

      return 0;
    }

    integer_type b_info(out_info_t &info) {
      // info.m = 10;
      // info.n = 10;
      // info.sparsity = bopt::Dense;

      // if (codegen) {
      //   // Evaluate as normal
      //   // b_cg(arg, res);
      // }

      return 0;
    }

    /**
     * @brief Update kinodynamic parameters and any parameters for the constraint
     *
     * @param model
     * @param data
     */
    void update(model_t &model, data_t &data, bopt::mathematical_program<double> &program) {
      // if(type == Numeric) task_->evaluate(model, data, J_, dJdq_);

      // // Update parameters
      // program.set_parameter(task_->parameters.w);
      // program.set_parameter(task_->parameters.xaccd);
    }

   private:
    // Numeric evaluation quantities
    matrix_t J_;
    matrix_t dJdq_;

    // Symbolic evaluation quantities

    // Codegen evaluation quantities
    // bopt::expression_evaluator<double> A_eval_;
    // bopt::expression_evaluator<double> b_eval_;

    std::shared_ptr<AbstractTask> task_;
  };

  void add_task(const std::string &name,
                const std::shared_ptr<FrameTaskNew> &task) {
    // Here we create the cost for it?

    // Create a new task and bind the necessary variables
    // std::vector<TaskCost> costs = {TaskCost(task)};
    // costs[0].update(model, data);
  }

  void getTaskID(const std::string &name) {

  }

  // Have collection of weights and stuff?

  // Error to compute for the task acceleration errors (xaccd = Kp e + Kd e_dot)
  // std::vector<pid_gains<double>> pid_gains;

  void init() {
    // All tasks added, finalise dynamics constraint
    qp_ = std::make_unique<bopt::solvers::qpoases_solver_instance>(program_);

    // Create solver
    is_initialised_ = true;
  }

  /**
   * @brief Loop function that computes the optimal control by solving an OCP
   *
   */
  void loop(const vector_t &q, const vector_t &v);

  /**
   * @brief Add a frame task to the program
   *
   * @param name
   * @param task
   */
  void add_task(const std::string &name,
                const std::shared_ptr<FrameTask> &task) {
    frame_tasks_index_map_.insert({name, frame_tasks_.size()});
    frame_tasks_.push_back(task);
    add_component_to_program(*task);
  }

  void add_task(const std::string &name,
                const std::shared_ptr<JointTrackingTask> &task) {
    joint_tracking_tasks_index_map_.insert(
        {name, joint_tracking_tasks_.size()});
    joint_tracking_tasks_.push_back(task);
    add_component_to_program(*task);
  }

  void add_task(const std::shared_ptr<CentreOfMassTask> &task) {
    com_task_ = task;
    add_component_to_program(*task);
  }

  std::shared_ptr<FrameTask> get_frame_task(const string_t &name) {
    if (frame_tasks_index_map_.find(name) != frame_tasks_index_map_.end()) {
      return frame_tasks_[frame_tasks_index_map_.at(name)];
    }
    return nullptr;
  }

  std::shared_ptr<JointTrackingTask> get_joint_tracking_task(
      const string_t &name) {
    if (joint_tracking_tasks_index_map_.find(name) !=
        joint_tracking_tasks_index_map_.end()) {
      return joint_tracking_tasks_[joint_tracking_tasks_index_map_.at(name)];
    }
    return nullptr;
  }

  std::shared_ptr<CentreOfMassTask> get_centre_of_mass_task() {
    return com_task_;
  }

  /**
   * @brief Generic cost 0.5 x^T H x + b^T x
   *
   */
  // class Cost : public bopt::quadratic_cost<double> {
  //  public:
  //   // Evaluate it numerically?
  //   void A() { task_->evaluate(); }

  //   // task_->evaluate(H, g)

  //  private:
  //   void update(model_t &model, data_t &data) {}

  //   std::shared_ptr<AbstractCost> task_;
  // };

  template <class CostType>
  void add_cost(const std::shared_ptr<CostType> &cost) {
    add_component_to_program(*cost);
  }

  class NoSlipConstraint : public bopt::linear_constraint<double> {};
  
  class NoSlipCost : public bopt::quadratic_cost<double> {};

  class FrictionConeConstraint : public bopt::linear_constraint<double> {};


  void add_contact_point(const std::string &name,
                         const std::shared_ptr<ContactPoint3D> &contact,
                         std::shared_ptr<SystemDynamics> &dynamics) {
    contact_3d_index_map_.insert({name, contact_3d_.size()});
    contact_3d_.push_back(contact);
    add_constraint(contact, dynamics);
  }

  std::shared_ptr<ContactPoint3D> get_contact_point_3d(const string_t &name) {
    if (contact_3d_index_map_.find(name) != contact_3d_index_map_.end()) {
      return contact_3d_[contact_3d_index_map_.at(name)];
    }
    return nullptr;
  }

  /**
   * @brief Adds a holonomic constraint to the program, accounting for the
   * mapping of the constraint forces to the system dynamics.
   *
   * @param constraint
   * @param dynamics
   */
  void add_constraint(const std::shared_ptr<HolonomicConstraint> &constraint,
                      std::shared_ptr<SystemDynamics> &dynamics) {
    // Add constraint to dynamics
    dynamics->add_constraint(*constraint);
    // Add to program
    add_component_to_program(*constraint);
  }

  class DynamicsConstraint : public bopt::linear_constraint<double> {};

  /**
   * @brief Registers the dynamics for the OSC, sets the number of constraint
   * forces applied to the system.
   *
   * @param dynamics
   */
  void add_dynamics(const std::shared_ptr<SystemDynamics> &dynamics) {
    // Add to program
    add_component_to_program(*dynamics);
  }

  std::vector<std::shared_ptr<Task>> get_all_tasks() {
    std::vector<std::shared_ptr<Task>> tasks = {};
    tasks.insert(tasks.end(), frame_tasks_.begin(), frame_tasks_.end());
    tasks.insert(tasks.end(), joint_tracking_tasks_.begin(),
                 joint_tracking_tasks_.end());
    if (com_task_) tasks.push_back(com_task_);
    return tasks;
  }

  std::vector<std::shared_ptr<ContactPoint>> get_all_contact_points() {
    std::vector<std::shared_ptr<ContactPoint>> contacts = {};
    contacts.insert(contacts.end(), contact_3d_.begin(), contact_3d_.end());
    return contacts;
  }

 private:
  // Model
  model_t model;
  // Symbolic model
  model_sym_t model_sym;

  osc_variables variables_;
  osc_parameters parameters_;

  bopt::mathematical_program<double> program_;

  // Initialisation flag
  bool is_initialised_ = false;

  // Tasks
  std::unordered_map<std::string, std::size_t> frame_tasks_index_map_;
  std::unordered_map<std::string, std::size_t> joint_tracking_tasks_index_map_;

  std::vector<std::shared_ptr<FrameTask>> frame_tasks_;
  std::vector<std::shared_ptr<JointTrackingTask>> joint_tracking_tasks_;
  std::shared_ptr<CentreOfMassTask> com_task_ = nullptr;

  // Contacts
  std::unordered_map<std::string, std::size_t> contact_3d_index_map_;

  std::vector<std::shared_ptr<ContactPoint3D>> contact_3d_;

  // Costs
  std::unordered_map<std::string, std::size_t> costs_index_map_;
  std::vector<std::shared_ptr<Cost>> costs_;

  template <class Component>
  void add_component_to_program(const Component &component) {
    component.add_to_program(*this);
  }

  template <class TaskType>
  void update_task(const model_t &model, const data_t &data, TaskType &task) {
    pid_error<double> e(task->dimension());
    task->compute_task_error(model, data, e);

    // Compute desired acceleration
    task->parameters().xacc_d = pid<double>::compute_error(task->pid, e);

    // Update parameters
    for (std::size_t i = 0; i < task->dimension(); ++i) {
      program_.set_parameter(task->parameters_v.w[i], task->parameters_d.w[i]);
      program_.set_parameter(task->parameters_v.xacc_d[i],
                             task->parameters_d.xacc_d[i]);
    }
  }

  template <class ContactPoint>
  void update_contact_point(const model_t &model, const data_t &data,
                            ContactPoint &contact) {
    // todo - detect a change in contact to minimise variable setting
    if (contact->in_contact) {
      for (std::size_t i = 0; i < contact->dimension(); ++i) {
        program_.set_parameter(contact->parameters_v.lambda_ub[i],
                               contact->parameters().lambda_ub[i]);

        program_.set_parameter(contact->parameters_v.lambda_lb[i],
                               contact->parameters().lambda_lb[i]);
      }

      for (std::size_t i = 0; i < 3; ++i) {
        program_.set_parameter(contact->parameters_v.n[i],
                               contact->parameters().n[i]);
        program_.set_parameter(contact->parameters_v.t[i],
                               contact->parameters().t[i]);
        program_.set_parameter(contact->parameters_v.b[i],
                               contact->parameters().b[i]);
      }
      program_.set_parameter(contact->parameters_v.mu,
                             contact->parameters().mu);
    } else {
      for (std::size_t i = 0; i < contact->dimension(); ++i) {
        program_.set_parameter(contact->parameters_v.lambda_ub[i], 0.0);

        program_.set_parameter(contact->parameters_v.lambda_lb[i], 0.0);
      }
    }
  }

  // QP solver
  std::unique_ptr<bopt::solvers::qpoases_solver_instance> qp_;
};

}  // namespace osc
