#pragma once

#include "osc/task.hpp"

/**
 * @brief Strict equality constraint for contact
 *
 */
struct contact_task_program_data {
    bopt::linear_constraint<double>::shared_ptr constraint;
    bopt::linear_constraint<double>::shared_ptr friction_cone;
    bopt::bounding_box_constraint<double>::shared_ptr friction_force_bounds;
};

template <typename Scalar>
struct contact_point_parameters {
    std::vector<Scalar> epsilon;

    Scalar mu;

    std::vector<Scalar> normal;
    std::vector<Scalar> tangent;

    std::vector<Scalar> friction_force_upper_bound;
    std::vector<Scalar> friction_force_lower_bound;
};

class ContactPoint {
   public:
    typedef double value_type;
    typedef std::size_t index_type;
    typedef int integer_type;

    typedef typename Eigen::VectorX<value_type> vector_t;

    ContactPoint() : dimension_(index_type(0)) {}

    bool in_contact = false;

    index_type dimension() const { return dimension_; }

    contact_point_parameters<bopt::variable> parameters_v;
    
    contact_point_parameters<value_type> &parameters() { return parameters_d; }
    
    // void add_to_program();

    contact_point_parameters<value_type> parameters_d;

   private:
    index_type dimension_;
};