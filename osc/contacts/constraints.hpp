#include "osc/contact/contact.hpp"

namespace osc {
/**
 * @brief Linearised friction cone constraint, approximates the friction cone as
 * an inscribed square pyramid, represented by a series of linear inequality
 * constraints.
 *
 */
class FrictionConeConstraint : public AbstractLinearConstraint {
 public:
  FrictionConeConstraint(
      const std::shared_ptr<AbstractFrictionContact> &contact)
      : contact_(contact) {
    // Create parameters
    mu = bopt::variable("mu");
    n = bopt::create_variable_vector("n", 3);
    t = bopt::create_variable_vector("t", 3);
    b = bopt::create_variable_vector("b", 3);
  }

  // Friction coeffcient
  bopt::variable mu;
  // Normal vector for contact (in contact surface frame)
  std::vector<bopt::variable> n;
  // Tangent vector for contact (in contact surface frame)
  std::vector<bopt::variable> t;
  // Remaining vector for contact (in contact surface frame)
  std::vector<bopt::variable> b;

  // Return linear constraint
  bopt::linear_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const override;

  // Associated friction contact with the constraint
  const std::shared_ptr<AbstractFrictionContact> &contact() const {
    return contact_;
  }

 private:
  std::shared_ptr<AbstractFrictionContact> contact_;
};

/**
 * @brief Bounding box constraints for friction forces on a given contact
 * 
 */
class FrictionForceConstraint : public AbstractBoundingBoxConstraint {
 public:
  FrictionForceConstraint(
      const std::shared_ptr<AbstractFrictionContact> &contact)
      : contact_(contact) {
    // Create parameters
    lb = bopt::create_variable_vector("lb", 3);
    ub = bopt::create_variable_vector("ub", 3);
  }

  // Normal vector for contact (in contact surface frame)
  std::vector<bopt::variable> ub;
  // Tangent vector for contact (in contact surface frame)
  std::vector<bopt::variable> lb;

  // Return bounding box constraint
  bopt::bounding_box_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const override;

  // Associated friction contact with the constraint
  const std::shared_ptr<AbstractFrictionContact> &contact() const {
    return contact_;
  }

 private:
  std::shared_ptr<AbstractFrictionContact> contact_;
};

/**
 * @brief No slip condition
 * 
 */
class NoSlipConstraint : public AbstractLinearConstraint {
 public:
  NoSlipConstraint(const std::shared_ptr<AbstractContact> &contact)
      : contact_(contact) {
    parameters.epsilon =
        bopt::create_variable_vector("eps", contact->dimension);
    epsilon.resize(contact->dimension);
    epsilon.setConstant(1e-3);
  }

  // Slack variable for no-slip condition in linear constraint defining
  // contact
  struct parameters {
    std::vector<bopt::variable> epsilon;
  };

  parameters parameters;

  vector_t epsilon;

  // Return linear constraint
  bopt::linear_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const override;

 private:
  std::shared_ptr<AbstractContact> contact_;
};
}