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
#include "osc/contact/constraints.hpp"
#include "osc/cost.hpp"
#include "osc/dynamics.hpp"
#include "osc/tasks/costs.hpp"

namespace osc {

class ProblemFormulation {
 public:
  // Collect tasks
  void add_task();
  void add_contact();
  void add_constraint();
  void add_cost();

 private:
};

class OSC {

 public:
  OSC(model_t &model, const index_t &nu);

  void add_variables(const std::vector<bopt::variable> &v) {
    program_.add_variables(v);
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
                std::vector<std::vector<bopt::variable>> &p) {
    program_.add_quadratic_cost(cost, {x}, p);
  }

  /**
   * @brief Add a generic constraint to the program
   *
   * @param constraint
   * @param x
   * @param p
   */
  void add_constraint(
      const std::shared_ptr<bopt::linear_constraint<double>> &constraint,
      const std::vector<bopt::variable> &x,
      const std::vector<std::vector<bopt::variable>> &p) {
    program_.add_linear_constraint(constraint, {x}, p);
  }

  void add_bounding_box_constraint(
      const std::shared_ptr<bopt::bounding_box_constraint<double>> &constraint,
      const std::vector<bopt::variable> &x,
      const std::vector<std::vector<bopt::variable>> &p) {
    program_.add_bounding_box_constraint(constraint, {x}, p);
  }

  /**
   * @brief Add a weighted task cost
   *
   * @param cost
   */
  void add_task_cost(const string_t &name,
                     const std::shared_ptr<WeightedTaskCost> &cost);

  std::shared_ptr<WeightedTaskCost> get_task_cost(const string_t &name) {
    if (task_costs_index_map_.find(name) != task_costs_index_map_.end()) {
      return task_costs_[task_costs_index_map_.at(name)];
    }
    return nullptr;
  }

  /**
   * @brief Adds variables related to the contact point
   *
   * @param contact
   */
  void add_contact(const std::shared_ptr<AbstractFrictionContact> &contact) {
    program_.add_variables(contact->lambda);
  }

  /**
   * @brief Add a no-slip condition constraint to the program
   *
   * @param constraint
   */
  void add_constraint(const std::shared_ptr<NoSlipConstraint> &constraint);

  void update_constraint(const std::shared_ptr<NoSlipConstraint> &constraint);

  /**
   * @brief Add a no-slip condition constraint to the program
   *
   * @param constraint
   */
  void add_constraint(
      const std::shared_ptr<FrictionForceConstraint> &constraint);

  void update_constraint(
      const std::shared_ptr<FrictionForceConstraint> &constraint);

  /**
   * @brief Add a friction cone constraint to the program.
   *
   * @param constraint
   */
  void add_constraint(
      const std::shared_ptr<FrictionConeConstraint> &constraint);

  /**
   * @brief Updates the parameters in the program for the given friction cone
   * constraint.
   *
   * @param constraint
   */
  void update_constraint(
      const std::shared_ptr<FrictionConeConstraint> &constraint);

  /**
   * @brief Add dynamics to the program.
   *
   * @param dynamics
   */
  void add_constraint(const std::shared_ptr<AbstractSystemDynamics> &dynamics);

  void init() {
    qp_ = std::make_unique<bopt::solvers::qpoases_solver_instance>(program_);

    // Create solver
    is_initialised_ = true;
  }

  /**
   * @brief Loop function that computes the optimal control by solving an OCP
   *
   */
  void loop(const vector_t &q, const vector_t &v);

  const osc_parameters &parameters() const { return parameters_; }
  const osc_variables &variables() const { return variables_; }

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
