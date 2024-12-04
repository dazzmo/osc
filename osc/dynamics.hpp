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

  const index_t &nu() const { return nu_; }

  void add_constraint(const std::shared_ptr<HolonomicConstraint> &constraint) {
    constraints_.push_back(constraint);
  }

  void add_dynamics(const std::shared_ptr<AbstractSystemDynamics> &dynamics) {
    dynamics_.push_back(dynamics);
  }

  /**
   * @brief Method to evaluate the system dynamics \dot x = f(x u), expressed in
   * the form \dot x - f(x, u) = 0. Typically this is used to compute the
   * unconstrained dynamics of the system.
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

  std::vector<bopt::variable> get_constraint_force_vector();

 private:
  index_t nu_;
  std::vector<std::shared_ptr<HolonomicConstraint>> constraints_;
  std::vector<std::shared_ptr<AbstractSystemDynamics>> dynamics_;
};

class SystemDynamicsConstraint : public AbstractLinearConstraint {
 public:
  SystemDynamicsConstraint(
      const std::shared_ptr<AbstractSystemDynamics> &dynamics)
      : dynamics_(dynamics) {}

  // Return linear constraint
  bopt::linear_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const;

 private:
  std::shared_ptr<AbstractSystemDynamics> dynamics_;
};

}  // namespace osc