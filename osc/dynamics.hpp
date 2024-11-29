#pragma once
#include "osc/common.hpp"
#include "osc/contact.hpp"
#include "osc/program.hpp"
#include "osc/holonomic.hpp"

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

  void add_constraint(const std::shared_ptr<HolonomicConstraint> &constraint) {
    constraints_.push_back(constraint);
  }

private:
  std::vector<std::shared_ptr<HolonomicConstraint>> constraints_;

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