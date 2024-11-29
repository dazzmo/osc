#pragma once
#include <pinocchio/algorithm/rnea.hpp>

#include "osc/common.hpp"
#include "osc/contact.hpp"
#include "osc/holonomic.hpp"
#include "osc/program.hpp"

namespace osc {

// Forward declarations
class OSC;
class AdditionalDynamics;

class AbstractDynamics {
 private:
  // Keep vector of contacts and constraints
  // void evaluate(model_t &model, data_t &data);
  // void evaluate(model_t &model, data_t &data);

};

class AbstractSystemDynamics {
 public:
  AbstractSystemDynamics(const model_t &model) : constraints_({}) {}

  void add_constraint(const std::shared_ptr<HolonomicConstraint> &constraint) {
    constraints_.push_back(constraint);
  }

  void add_dynamics(const std::shared_ptr<AbstractSystemDynamics> &dynamics) {
    dynamics_.push_back(dynamics);
  }

  virtual vector_sym_t evaluate(const model_sym_t &model, data_sym_t &data,
                        const vector_sym_t &q, const vector_sym_t &v,
                        const vector_sym_t &a) const {
    throw std::runtime_error("No symbolic evaluate implemented for dynamics");
  }

  // Return linear constraint
  bopt::linear_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const {
    // Compute the target frame in the contact frame of the model
    vector_sym_t q = create_symbolic_vector("q", model.nq);
    vector_sym_t v = create_symbolic_vector("v", model.nv);
    vector_sym_t a = create_symbolic_vector("a", model.nv);

    // Compute frame state
    model_sym_t model_sym = model.cast<sym_t>();
    data_sym_t data_sym(model_sym);

    // Compute the kinematic tree state of the system
    pinocchio::forwardKinematics(model_sym, data_sym, q, v, a);
    pinocchio::updateFramePlacements(model_sym, data_sym);

    vector_sym_t f;
    f = pinocchio::rnea(model_sym, data_sym, q, v, a);

    // Add additional dynamics
    for (const auto &dyn : dynamics_) {
      f += dyn->evaluate(model_sym, data_sym, q, v, a);
    }

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
    vector_sym_t x(model.nv + l.size());
    x << a, l;

    return bopt::casadi::linear_constraint<double>::create(
        casadi::eigen_to_casadi(f), casadi::eigen_to_casadi(x),
        sym_vector_t({casadi::eigen_to_casadi(q), casadi::eigen_to_casadi(v)}),
        bopt::bound_type::Equality);
  }

 private:
  std::vector<std::shared_ptr<HolonomicConstraint>> constraints_;
  std::vector<std::shared_ptr<AbstractSystemDynamics>> dynamics_;
};

class SystemDynamics : public OSCComponent {
  friend class OSC;
  friend class AdditionalDynamics;

 public:
  typedef double value_type;

  SystemDynamics(const model_t &model, const index_t &nu);

  /**
   * @brief Number of constraint forces acting on the system
   *
   * @return index_t
   */
  index_t nf() const { return f.size(); }

  void add_constraint(const HolonomicConstraint &constraint);

  void add_additional_dynamics(AdditionalDynamics &dynamics);

  void add_to_program(OSC &osc_program) const;

 private:
  vector_sym_t q;
  vector_sym_t v;
  vector_sym_t a;
  vector_sym_t u;
  vector_sym_t f;

  model_sym_t model;

  // Symbolic representation of the system dynamics
  vector_sym_t dynamics_;
};

class AdditionalDynamics {
  friend class SystemDynamics;

 protected:
  const vector_sym_t &get_q(SystemDynamics &dynamics) const {
    return dynamics.q;
  }
  const vector_sym_t &get_v(SystemDynamics &dynamics) const {
    return dynamics.v;
  }
  const vector_sym_t &get_a(SystemDynamics &dynamics) const {
    return dynamics.a;
  }
  const vector_sym_t &get_u(SystemDynamics &dynamics) const {
    return dynamics.u;
  }
  const vector_sym_t &get_f(SystemDynamics &dynamics) const {
    return dynamics.f;
  }
  vector_sym_t &get_dynamics(SystemDynamics &dynamics) const {
    return dynamics.dynamics_;
  }
  model_sym_t &get_model(SystemDynamics &dynamics) const {
    return dynamics.model;
  }

 public:
  virtual void add_to_dynamics(SystemDynamics &dynamics) const = 0;

 private:
};

}  // namespace osc