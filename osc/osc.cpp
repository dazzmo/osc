#include "osc/osc.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

OSC::OSC(model_t &model, const index_t &nu)
    : model(model), model_sym(model.cast<sym_t>()), program_() {
  // Create system dynamics
  variables_.a = bopt::create_variable_vector("a", model.nv);
  variables_.u = bopt::create_variable_vector("u", nu);
  parameters_.q = bopt::create_variable_vector("q", model.nq);
  parameters_.v = bopt::create_variable_vector("v", model.nv);

  // Add variables to the program
  program_.add_variables(variables_.a);
  program_.add_variables(variables_.u);
  program_.add_parameters(parameters_.q);
  program_.add_parameters(parameters_.v);
}


void OSC::add_task_cost(const std::shared_ptr<WeightedTaskCost> &cost) {
  task_costs_index_map_.insert({name, task_costs_.size()});
  task_costs_.push_back(cost);

  // Add parameters to the program
  program_.add_parameters(cost->parameters.w);
  program_.add_parameters(cost->parameters.x);

  update_task_cost(cost);

  // Add cost to the program
  program_.add_quadratic_cost(
      cost->to_cost(model), {variables_.a},
      {parameters_.q, parameters_.v, cost->parameters.w, cost->parameters.x});
}

void OSC::update_task_cost(const std::shared_ptr<WeightedTaskCost> &cost) {
  for (auto i = 0; i < cost->task()->dimension; ++i) {
    program_.set_parameter(cost->parameters.w[i], cost->w[i]);
  }
}

void OSC::add_constraint(
    const std::shared_ptr<AbstractSystemDynamics> &dynamics) {
  // Collect constraint force variables
  std::vector<bopt::variable> lambda = dynamics->get_constraint_force_vector();

  std::vector<bopt::variable> x = {};
  x.insert(x.end(), variables_.a.begin(), variables_.a.end());
  x.insert(x.end(), variables_.u.begin(), variables_.u.end());
  x.insert(x.end(), lambda.begin(), lambda.end());

  program_.add_linear_constraint(dynamics->to_constraint(model), {x},
                                 {parameters_.q, parameters_.v});
}

void OSC::add_constraint(const std::shared_ptr<NoSlipConstraint> &constraint) {
  // Add slack variables
  program_.add_parameters(constraint->parameters.epsilon);

  update_constraint(constraint);

  program_.add_linear_constraint(
      constraint->to_constraint(model), {variables_.a},
      {parameters_.q, parameters_.v, constraint->parameters.epsilon});
}

void OSC::update_constraint(
    const std::shared_ptr<NoSlipConstraint> &constraint) {
  // Add slack variables
  for (int i = 0; i < constraint->contact()->dimension; ++i) {
    program_.set_parameter(constraint->parameters.epsilon[i],
                           constraint->epsilon[i]);
  }
}

void OSC::add_constraint(
    const std::shared_ptr<FrictionForceConstraint> &constraint) {
  program_.add_parameters(constraint->ub);
  program_.add_parameters(constraint->lb);

  update_constraint(constraint);

  program_.add_bounding_box_constraint(constraint->to_constraint(model),
                                       {constraint->contact()->lambda},
                                       {constraint->lb, constraint->ub});
}

void OSC::update_constraint(
    const std::shared_ptr<FrictionForceConstraint> &constraint) {
  for (int i = 0; i < constraint->contact()->dimension; ++i) {
    program_.set_parameter(constraint->ub[i], constraint->contact()->ub[i]);
    program_.set_parameter(constraint->lb[i], constraint->contact()->lb[i]);
  }
}

void OSC::add_constraint(
    const std::shared_ptr<FrictionConeConstraint> &constraint) {
  // Add parameters to the program
  program_.add_parameters(constraint->n);
  program_.add_parameters(constraint->t);
  program_.add_parameters(constraint->b);
  program_.add_parameter(constraint->mu);

  update_constraint(constraint);

  // Add constraint to program
  program_.add_linear_constraint(
      constraint->to_constraint(model), {constraint->contact()->lambda},
      {constraint->n, constraint->t, constraint->b, {constraint->mu}});
}

void OSC::update_constraint(
    const std::shared_ptr<FrictionConeConstraint> &constraint) {
  for (int i = 0; i < 3; ++i) {
    program_.set_parameter(constraint->n[i], constraint->contact()->n[i]);
    program_.set_parameter(constraint->t[i], constraint->contact()->t[i]);
    program_.set_parameter(constraint->b[i], constraint->contact()->b[i]);
  }
  program_.set_parameter(constraint->mu, constraint->contact()->mu);
}

void OSC::loop(const vector_t &q, const vector_t &v) {
  assert(is_initialised_ && "OSC is not initialised!");
  // Update references for all tasks
  for (auto i = 0; i < q.size(); ++i) {
    program_.set_parameter(parameters_.q[i], q[i]);
  }
  for (auto i = 0; i < v.size(); ++i) {
    program_.set_parameter(parameters_.v[i], v[i]);
  }

  data_t data(model);

  // Update pinocchio model for computations
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::centerOfMass(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  // Update task errors
  for (auto &cost : task_costs_) {
    pid_error<double> e(cost->task()->dimension);
    cost->task()->evaluate_error(model, data, e.error, e.error_dot);
    // Update desired task acceleration
    vector_t a = pid<double>::compute_error(cost->pid, e);
    // Update desired task error
    for (auto i = 0; i < cost->task()->dimension; ++i) {
      program_.set_parameter(cost->parameters.x[i], a[i]);
    }
  }

  // Once updated, solve the program
  qp_->solve();

  // if(reset) qp.reset()
}

}  // namespace osc