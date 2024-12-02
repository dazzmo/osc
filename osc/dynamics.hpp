#pragma once
#include <pinocchio/algorithm/rnea.hpp>

#include "osc/common.hpp"
#include "osc/contact.hpp"
#include "osc/holonomic.hpp"

namespace osc {

class AbstractSystemDynamics {
 public:
  AbstractSystemDynamics(const model_t &model, const index_t &nu)
      : nu_(nu), constraints_({}) {}

  void add_constraint(const std::shared_ptr<HolonomicConstraint> &constraint) {
    constraints_.push_back(constraint);
  }

  void add_dynamics(const std::shared_ptr<AbstractSystemDynamics> &dynamics) {
    dynamics_.push_back(dynamics);
  }

  /**
   * @brief Method to evaluate the system dynamics \dot x = f(x u), expressed in
   * the form \dot x - f(x, u) = 0. Typically this is used to compute the unconstrained dynamics of the system.
   *
   * @param model
   * @param data
   * @param q Generalised position
   * @param v Generalised velocity
   * @param a Generalised acceleration
   * @param u Control input vector
   * @return vector_sym_t
   */
  virtual vector_sym_t evaluate(const model_sym_t &model, data_sym_t &data,
                                const vector_sym_t &q, const vector_sym_t &v,
                                const vector_sym_t &a,
                                const vector_sym_t &u) const {
    throw std::runtime_error("No symbolic evaluate implemented for dynamics");
  }

  // Return linear constraint
  bopt::linear_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const {
    // Compute the target frame in the contact frame of the model
    vector_sym_t q = create_symbolic_vector("q", model.nq);
    vector_sym_t v = create_symbolic_vector("v", model.nv);
    vector_sym_t a = create_symbolic_vector("a", model.nv);
    vector_sym_t u = create_symbolic_vector("u", nu_);

    // Compute frame state
    model_sym_t model_sym = model.cast<sym_t>();
    data_sym_t data_sym(model_sym);

    // Compute the kinematic tree state of the system
    pinocchio::forwardKinematics(model_sym, data_sym, q, v, a);
    pinocchio::updateFramePlacements(model_sym, data_sym);

    // Create unconstrained dynamics
    vector_sym_t f;
    f = this->evaluate(model_sym, data_sym, q, v, a, u);

    // Add additional dynamics
    for (const auto &dyn : dynamics_) {
      f += dyn->evaluate(model_sym, data_sym, q, v, a, u);
    }

    // Add constraint forces
    vector_sym_t l;
    for (const auto &constraint : constraints_) {
      vector_sym_t lambda_i =
          create_symbolic_vector("l", constraint->dimension);
      matrix_sym_t J_i(constraint->dimension, model.nv);
      constraint->jacobian(model_sym, data_sym, J_i);
      f -= J_i.transpose() * lambda_i;
      // Add to force variables
      l.conservativeResize(l.size() + lambda_i.size());
      l.bottomRows(lambda_i.size()) = lambda_i;
    }

    // Create vector for linearisation
    vector_sym_t x(model.nv + nu_ + l.size());
    x << a, u, l;

    return bopt::casadi::linear_constraint<double>::create(
        casadi::eigen_to_casadi(f), casadi::eigen_to_casadi(x),
        sym_vector_t({casadi::eigen_to_casadi(q), casadi::eigen_to_casadi(v)}),
        bopt::bound_type::Equality);
  }

 private:
  index_t nu_;
  std::vector<std::shared_ptr<HolonomicConstraint>> constraints_;
  std::vector<std::shared_ptr<AbstractSystemDynamics>> dynamics_;
};

}  // namespace osc