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

  void add_parameters(const std::vector<bopt::variable> &p) {
    program_.add_parameters(p);
  }

  /**
   * @brief Add a generic quadratic cost the program that doesn't fall under any
   * of the categories provided by OSC.
   *
   * @param x
   * @param p
   */
  void add_cost(const std::shared_ptr<bopt::quadratic_cost<double>> &cost,
                std::vector<bopt::variable> &x,
                std::vector<std::vector<bopt::variable>> &p) {}

  /**
   * @brief Add a generic constraint to the program
   *
   * @param constraint
   * @param x
   * @param p
   */
  void add_constraint(
      const std::shared_ptr<bopt::linear_constraint<double>> &constraint,
      std::vector<bopt::variable> &x,
      std::vector<std::vector<bopt::variable>> &p) {}

  /**
   * @brief Add a frame task to the program
   *
   * @param name
   * @param task
   */
  void add_task(const string_t &name,
                const std::shared_ptr<FrameTask> &task) {
    frame_tasks_index_map_.insert({name, frame_tasks_.size()});
    frame_tasks_.push_back(task);
  }

  std::shared_ptr<FrameTask> get_frame_task(const string_t &name) {
    if (frame_tasks_index_map_.find(name) != frame_tasks_index_map_.end()) {
      return frame_tasks_[frame_tasks_index_map_.at(name)];
    }
    return nullptr;
  }

  /**
   * @brief Add a weighted task cost
   *
   * @param cost
   */
  void add_task_cost(const string_t &name,
                     const std::shared_ptr<WeightedTaskCost> &cost) {
    task_costs_index_map_.insert({name, task_costs_.size()});
    task_costs_.push_back(cost);

    // Add parameters to the program
    program_.add_parameters(cost->parameters.w);
    program_.add_parameters(cost->parameters.x);

    // Add cost to the program
    program_.add_quadratic_cost(
        cost->to_cost(model), {variables_.a},
        {parameters_.q, parameters_.v, cost->parameters.w, cost->parameters.x});
  }

  std::shared_ptr<WeightedTaskCost> get_task_cost(const string_t &name) {
    if (task_costs_index_map_.find(name) != task_costs_index_map_.end()) {
      return task_costs_[task_costs_index_map_.at(name)];
    }
    return nullptr;
  }

  /**
   * @brief Add a contact point for the system
   *
   * @param contact
   */
  void add_contact(const std::shared_ptr<AbstractFrictionContact> &contact) {
    program_.add_variables(contact->lambda);
  }

  // std::shared_ptr<AbstractFrictionContact> get_contact(const string_t &name)
  // {}

  /**
   * @brief Add a no-slip condition constraint to the program
   *
   * @param constraint
   */
  void add_no_slip_constraint(
      const std::shared_ptr<NoSlipConstraint> &constraint) {
    // Add slack variables
    program_.add_parameters(constraint->epsilon);

    program_.add_linear_constraint(
        constraint->to_constraint(model), {variables_.a},
        {parameters_.q, parameters_.v, constraint->epsilon});
  }

  /**
   * @brief Add dynamics to the program.
   *
   * @param dynamics
   */
  void add_dynamics_constraint(
      const std::shared_ptr<AbstractSystemDynamics> &dynamics) {
    // Create vector for linear expression

    // todo - Create constraint force vector in same order as in dynamics

    std::vector<bopt::variable> x = {};
    x.insert(x.end(), variables_.a.begin(), variables_.a.end());
    x.insert(x.end(), variables_.u.begin(), variables_.u.end());
    x.insert(x.end(), variables_.lambda.begin(), variables_.lambda.end());

    program_.add_linear_constraint(dynamics->to_constraint(model), {x},
                                   {parameters_.q, parameters_.v});
  }

  /**
   * @brief Add a friction cone constraint to the program.
   *
   * @param constraint
   */
  void add_friction_constraint(
      const std::shared_ptr<FrictionConeConstraint> &constraint) {
    // Add parameters to the program
    program_.add_parameters(constraint->n);
    program_.add_parameters(constraint->t);
    program_.add_parameters(constraint->b);
    program_.add_parameter(constraint->mu);

    // Add constraint to program
    program_.add_linear_constraint(
        constraint->to_constraint(model), {constraint->contact()->lambda},
        {constraint->n, constraint->t, constraint->b, {constraint->mu}});
  }

  // void update_friction_constraint(const string_t &name) {
  //   // const auto constraint = find()...

  //   for (int i = 0; i < 3; ++i) {
  //     program_.set_parameter(constraint->n[i], constraint->contact()->n[i]);
  //     program_.set_parameter(constraint->t[i], constraint->contact()->t[i]);
  //     program_.set_parameter(constraint->b[i], constraint->contact()->b[i]);
  //   }

  //   // Update any other features
  // }

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

 private:
  // Model
  model_t model;
  // Symbolic model
  model_sym_t model_sym;

  // Variables within the program
  osc_variables variables_;
  // Parameters within the program
  osc_parameters parameters_;

  bopt::mathematical_program<double> program_;

  // Initialisation flag
  bool is_initialised_ = false;

  // Tasks
  std::unordered_map<std::string, std::size_t> frame_tasks_index_map_;
  std::unordered_map<std::string, std::size_t> joint_tracking_tasks_index_map_;

  std::vector<std::shared_ptr<FrameTask>> frame_tasks_;

  // Task Costs
  std::vector<std::shared_ptr<WeightedTaskCost>> task_costs_;
  std::unordered_map<std::string, std::size_t> task_costs_index_map_;

  // Friction Cone Constraints
  std::unordered_map<std::string, std::size_t> friction_cone_index_map_;
  std::vector<std::shared_ptr<FrictionConeConstraint>>
      friction_cone_constraints;

  // No-slip constraints
  std::unordered_map<std::string, std::size_t> no_slip_index_map_;
  std::vector<std::shared_ptr<NoSlipConstraint>> no_slip_constraints;

  // QP solver
  std::unique_ptr<bopt::solvers::qpoases_solver_instance> qp_;
};

}  // namespace osc
