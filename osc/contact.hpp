#pragma once

#include "osc/task.hpp"
#include "osc/constraint.hpp"

namespace osc {

struct contact_point_traits {};

template <typename VectorType, typename ScalarType>
struct contact_point_parameters {
    VectorType epsilon;

    ScalarType mu;

    VectorType n;
    VectorType t;
    VectorType b;

    VectorType friction_force_upper_bound;
    VectorType friction_force_lower_bound;
};

class ContactPoint : public HolonomicConstraint {
   public:
    friend class OSC;

    typedef double value_type;
    typedef std::size_t index_type;
    typedef int integer_type;

    typedef typename Eigen::VectorX<value_type> vector_t;

    ContactPoint(const index_type &dim, const index_type &model_nq,
                 const index_type &model_nv, const string_t &target)
        : HolonomicConstraint(dim, model_nq, model_nv), target_frame(target) {
        parameters_v.epsilon = create_variable_vector("eps", dimension());
        parameters_d.epsilon = eigen_vector_t::Zero(dimension());

        parameters_v.n = create_variable_vector("n", dimension());
        parameters_d.n = eigen_vector_t::Zero(dimension());

        parameters_v.t = create_variable_vector("t", dimension());
        parameters_d.t = eigen_vector_t::Zero(dimension());

        parameters_v.b = create_variable_vector("b", dimension());
        parameters_d.b = eigen_vector_t::Zero(dimension());

        parameters_v.friction_force_upper_bound =
            create_variable_vector("friction_force_upper_bound", dimension());
        parameters_d.friction_force_upper_bound =
            eigen_vector_t::Zero(dimension());

        parameters_v.friction_force_lower_bound =
            create_variable_vector("friction_force_lower_bound", dimension());
        parameters_d.friction_force_lower_bound =
            eigen_vector_t::Zero(dimension());
    }

    virtual bopt::linear_constraint<value_type>::shared_ptr
    create_friction_constraint(const model_sym_t &model) const = 0;

    virtual bopt::bounding_box_constraint<value_type>::shared_ptr
    create_friction_bound_constraint(const model_sym_t &model) const = 0;

    virtual bopt::linear_constraint<value_type>::shared_ptr
    create_no_slip_constraint(const model_sym_t &model) const = 0;

    /**
     * @brief Provides the contact Jacobian for the system
     *
     */
    eigen_matrix_sym_t constraint_jacobian(
        const model_sym_t &model, const eigen_vector_sym_t &q) const override {
        pinocchio::DataTpl<sym_t> data(model);

        // Compute the kinematic tree state of the system
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        eigen_matrix_sym_t J = eigen_matrix_sym_t::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q,
                                        model.getFrameId(target_frame),
                                        pinocchio::ReferenceFrame::WORLD, J);

        return J;
    }

    bool in_contact = false;

    string_t target_frame;

    contact_point_parameters<eigen_vector_t, value_type> &parameters() {
        return parameters_d;
    }

   protected:
    contact_point_parameters<eigen_vector_t, value_type> parameters_d;
    contact_point_parameters<eigen_vector_var_t, bopt::variable> parameters_v;
};

class ContactPoint3D : public ContactPoint {
   public:
    ContactPoint3D(const model_sym_t &model, const string_t &target)
        : ContactPoint(3, model.nq, model.nv, target) {}

    bopt::linear_constraint<value_type>::shared_ptr create_friction_constraint(
        const model_sym_t &model) const override;

    bopt::bounding_box_constraint<value_type>::shared_ptr
    create_friction_bound_constraint(const model_sym_t &model) const override;

    bopt::linear_constraint<value_type>::shared_ptr create_no_slip_constraint(
        const model_sym_t &model) const override;

   private:
};

}  // namespace osc