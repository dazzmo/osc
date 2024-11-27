#pragma once

#include "osc/constraint.hpp"
#include "osc/program.hpp"
#include "osc/task.hpp"

namespace osc {

class OSC;

struct contact_point_traits {};

template <typename VectorType, typename ScalarType>
struct contact_point_parameters {
  // Slack variable for no-slip condition in linear constraint defining
  // contact
  VectorType epsilon;

  // Friction coeffcient
  ScalarType mu;
  // Normal vector for contact (in contact surface frame)
  VectorType n;
  // Tangent vector for contact (in contact surface frame)
  VectorType t;
  // Remaining vector for contact (in contact surface frame)
  VectorType b;

  // Upper bound for frictional force
  VectorType lambda_ub;
  // Lower bound for frictional force
  VectorType lambda_lb;
};

class ContactPoint : public HolonomicConstraint {
 public:
  friend class OSC;

  typedef double value_type;
  typedef std::size_t index_type;
  typedef int integer_type;

  typedef typename Eigen::VectorX<value_type> vector_t;

  ContactPoint(const index_type &dim, const index_type &model_nq,
               const index_type &model_nv, const string_t &target);

  virtual bopt::linear_constraint<value_type>::shared_ptr
  create_friction_constraint(const model_sym_t &model) const = 0;

  virtual bopt::bounding_box_constraint<value_type>::shared_ptr
  create_friction_bound_constraint(const model_sym_t &model) const = 0;

  /**
   * @brief Provides the contact Jacobian for the system
   *
   */
  matrix_sym_t constraint_jacobian(const model_sym_t &model,
                                   const vector_sym_t &q) const override;

  bool in_contact = false;

  string_t frame;

  contact_point_parameters<vector_t, value_type> &parameters() {
    return parameters_d;
  }

 protected:
  contact_point_parameters<vector_t, value_type> parameters_d;
  contact_point_parameters<std::vector<bopt::variable>, bopt::variable>
      parameters_v;
};

class ContactPoint3D : public ContactPoint {
 public:
  ContactPoint3D(const model_t &model, const string_t &frame)
      : ContactPoint(3, model.nq, model.nv, frame) {}

  static std::shared_ptr<ContactPoint3D> create(const model_t &model,
                                                const string_t &frame) {
    return std::make_shared<ContactPoint3D>(model, frame);
  }

  /**
   * @brief Imposes a linearised friction cone constraint for the contact
   * forces \f$ \lambda \f$
   *
   * @param model
   * @return bopt::linear_constraint<value_type>::shared_ptr
   */
  bopt::linear_constraint<value_type>::shared_ptr create_friction_constraint(
      const model_sym_t &model) const override;

  /**
   * @brief Creates a simple bounding box constraint for the constraint forces
   * \f$ \lambda \f$
   *
   * @param model
   * @return bopt::bounding_box_constraint<value_type>::shared_ptr
   */
  bopt::bounding_box_constraint<value_type>::shared_ptr
  create_friction_bound_constraint(const model_sym_t &model) const override;

  /**
   * @brief The no-slip condition imposed as the linear constraint \f$ J \ddot
   * q +
   * \dot J \dot q - \epsilon = 0 \f$
   *
   * @param model
   * @return bopt::linear_constraint<value_type>::shared_ptr
   */
  bopt::linear_constraint<value_type>::shared_ptr create_no_slip_constraint(
      const model_sym_t &model) const;

  matrix_sym_t constraint_jacobian(const model_sym_t &model,
                                   const vector_sym_t &q) const {
    return ContactPoint::constraint_jacobian(model, q).topRows(3);
  }

 protected:
  /**
   * @brief Registers the contact as a constraint within the provided program.
   *
   * @param model
   * @param osc_program
   *
   * @note This assumes that constraint forces lambda have been added to the
   * program before this is called, such that the forces used will be the last
   * dimension() variables of lambda in the program.
   */
  void add_to_program(OSC &osc_program) const override;

 private:
};

}  // namespace osc