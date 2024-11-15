#pragma once

#include "osc/task.hpp"

/**
 * @brief Strict equality constraint for contact
 * 
 */
struct contact_task_program_data {
    bopt::linear_constraint<double>::shared_ptr constraint;
    bopt::linear_constraint<double>::shared_ptr friction_cone;
};

struct contact_task_relaxed_program_data : contact_task_program_data {
    bopt::quadratic_cost<double>::shared_ptr cost;
    std::vector<bopt::variable> epsilon;
};

class ContactTaskSoft {
   public:
    typedef double value_type;
    typedef std::size_t index_type;
    typedef int integer_type;

    typedef typename Eigen::VectorX<value_type> vector_t;

    ContactTask() : dimension_(index_type(0)) {}

    index_type dimension() const { return dimension_; }

    value_type weight() {}

    integer_type pid_gains(vector_t &p, vector_t &i, vector_t &d) {
        pid.p = p;
        pid.i = i;
        pid.d = d;
    }

    virtual integer_type get_state(task_state &state) {}
    virtual integer_type get_error(task_error &error) {}

    integer_type set_reference(const task_state &reference) {}

    task_parameters parameters;
    pid_gains<Eigen::Vector3d> pid;

    contact_task_variables variables;

   private:
    index_type dimension_;
};

// void add_contact_to_dynamics()

// void to_contact_cost(const Task &task) {
//     // Create a cost for this task

//     // Create model and evaluate
// }

// void to_contact_constraint(const Task &task) {
//     // Create a cost for this task

//     // Create model and evaluate
// }