#pragma once

#include <bopt/program.hpp>
#include <bopt/solvers/qpoases.hpp>

#include "osc/contact.hpp"
#include "osc/dynamics.hpp"
#include "osc/task.hpp"

namespace osc {

/**
 * @brief Visitor class that can be customised and added to a typical OSC
 * instance
 *
 */
// class osc_visitor {
//     void init();
//     void update_references();
//     void update_parameters();
// };

struct osc_task_maps {
    template <typename TaskType>
    using task_map_t = std::unordered_map<string_t, std::shared_ptr<TaskType>>;

    task_map_t<PositionTask> position_tasks_;
    task_map_t<OrientationTask> orientation_tasks_;
    task_map_t<SE3Task> se3_tasks_;
    task_map_t<Task> generic_tasks_;
};

struct osc_contact_point_maps {
    template <typename ContactPointType>
    using task_map_t =
        std::unordered_map<string_t, std::shared_ptr<ContactPointType>>;

    task_map_t<ContactPoint3D> contact_3d_;
};

template <typename VectorType>
struct osc_variables {
    VectorType a;
    VectorType u;
    VectorType lambda;
};

template <typename VectorType>
struct osc_parameters {
    VectorType q;
    VectorType v;
};

class OSC {
   public:
    typedef state<eigen_vector_t> model_state_t;

    OSC(model_sym_t &model) : model(model), dynamics_(model) {
        // Create system dynamics
        variables.a = create_variable_vector("a", model.nv);

        parameters.q = create_variable_vector("q", model.nq);
        parameters.v = create_variable_vector("v", model.nv);

        // Add variables to the program
        for (std::size_t i = 0; i < model.nv; ++i) {
            program.add_variable(variables.a[i]);
        }
        for (std::size_t i = 0; i < model.nq; ++i) {
            program.add_parameter(parameters.q[i]);
        }
        for (std::size_t i = 0; i < model.nv; ++i) {
            program.add_parameter(parameters.v[i]);
        }
    }

    void init() {
        // All tasks added, finalise dynamics constraint
        add_dynamics_to_program(dynamics_);
        // visitor.init();

        qp_ = std::make_unique<bopt::solvers::qpoases_solver_instance>(program);

        // Create solver
        is_initialised_ = true;
    }

    /**
     * @brief Loop function that computes the optimal control by solving an OCP
     *
     */
    void loop(const state<eigen_vector_t> &model_state);

    void add_position_task(const std::string &name,
                           std::shared_ptr<PositionTask> &task) {
        tasks_.position_tasks_.insert({name, task});
        add_task_to_program(*task);
    }

    std::shared_ptr<PositionTask> get_position_task(const string_t &name) {
        if (tasks_.position_tasks_.find(name) != tasks_.position_tasks_.end()) {
            return tasks_.position_tasks_.at(name);
        }
        return nullptr;
    }

    void add_orientation_task(const std::string &name,
                              std::shared_ptr<OrientationTask> &task) {
        tasks_.orientation_tasks_.insert({name, task});
        add_task_to_program(*task);
    }

    std::shared_ptr<OrientationTask> get_orientation_task(
        const string_t &name) {
        if (tasks_.orientation_tasks_.find(name) !=
            tasks_.orientation_tasks_.end()) {
            return tasks_.orientation_tasks_.at(name);
        }
        return nullptr;
    }

    void add_se3_task(const std::string &name, std::shared_ptr<SE3Task> &task) {
        tasks_.se3_tasks_.insert({name, task});
        add_task_to_program(*task);
    }

    std::shared_ptr<SE3Task> get_se3_task(const string_t &name) {
        if (tasks_.se3_tasks_.find(name) != tasks_.se3_tasks_.end()) {
            return tasks_.se3_tasks_.at(name);
        }
        return nullptr;
    }

    template <class TaskType>
    void add_task_to_program(const TaskType &task) {
        auto cost = task.to_task_cost(model);

        // program.add_parameter();
        for (std::size_t i = 0; i < task.parameters_v.w.size(); ++i) {
            program.add_parameter(task.parameters_v.w[i]);
        }
        for (std::size_t i = 0;
             i < task.parameters_v.desired_task_acceleration.size(); ++i) {
            program.add_parameter(
                task.parameters_v.desired_task_acceleration[i]);
        }

        // Bind to program
        program.add_quadratic_cost(
            cost,
            // Variables
            {eigen_to_std_vector<bopt::variable>::convert(variables.a)},
            // Parameters
            {eigen_to_std_vector<bopt::variable>::convert(parameters.q),
             eigen_to_std_vector<bopt::variable>::convert(parameters.v),
             // Task-specific parameters
             eigen_to_std_vector<bopt::variable>::convert(task.parameters_v.w),
             eigen_to_std_vector<bopt::variable>::convert(
                 task.parameters_v.desired_task_acceleration)});
    }

    void add_contact_point_3d(const std::string &name,
                              std::shared_ptr<ContactPoint3D> &contact) {
        contacts_.contact_3d_.insert({name, contact});
        add_contact_point_to_program(*contact);
    }

    template <class ContactType>
    void add_contact_point_to_program(const ContactType &contact) {
        // Add constraint components
        add_holonomic_constraint_to_program(contact);

        // Parameters
        for (std::size_t i = 0; i < contact.parameters_v.epsilon.size(); ++i) {
            program.add_parameter(contact.parameters_v.epsilon[i]);
        }
        for (std::size_t i = 0; i < contact.parameters_v.n.size(); ++i) {
            program.add_parameter(contact.parameters_v.n[i]);
        }
        for (std::size_t i = 0; i < contact.parameters_v.t.size(); ++i) {
            program.add_parameter(contact.parameters_v.t[i]);
        }
        for (std::size_t i = 0; i < contact.parameters_v.b.size(); ++i) {
            program.add_parameter(contact.parameters_v.b[i]);
        }
        for (std::size_t i = 0;
             i < contact.parameters_v.friction_force_upper_bound.size(); ++i) {
            program.add_parameter(
                contact.parameters_v.friction_force_upper_bound[i]);
        }
        for (std::size_t i = 0;
             i < contact.parameters_v.friction_force_lower_bound.size(); ++i) {
            program.add_parameter(
                contact.parameters_v.friction_force_lower_bound[i]);
        }
        program.add_parameter(contact.parameters_v.mu);

        // Create constraints
        auto friction_cone = contact.create_friction_constraint(model);
        auto friction_bound = contact.create_friction_bound_constraint(model);
        auto no_slip = contact.create_no_slip_constraint(model);

        // Bind to program
        program.add_linear_constraint(
            friction_cone,
            // Variables
            {eigen_to_std_vector<bopt::variable>::convert(
                variables.lambda.bottomRows(contact.dimension()))},
            // contact-specific parameters
            {eigen_to_std_vector<bopt::variable>::convert(
                 contact.parameters_v.n),
             eigen_to_std_vector<bopt::variable>::convert(
                 contact.parameters_v.t),
             eigen_to_std_vector<bopt::variable>::convert(
                 contact.parameters_v.b),
             {contact.parameters_v.mu}});

        program.add_linear_constraint(
            no_slip,
            // Variables
            {eigen_to_std_vector<bopt::variable>::convert(variables.a)},
            // Parameters
            {eigen_to_std_vector<bopt::variable>::convert(parameters.q),
             eigen_to_std_vector<bopt::variable>::convert(parameters.v),
             // Task-specific parameters
             eigen_to_std_vector<bopt::variable>::convert(
                 contact.parameters_v.epsilon)});

        program.add_bounding_box_constraint(
            friction_bound,
            // Variables
            {eigen_to_std_vector<bopt::variable>::convert(
                variables.lambda.bottomRows(contact.dimension()))},
            // Parameters
            {eigen_to_std_vector<bopt::variable>::convert(
                 contact.parameters_v.friction_force_lower_bound),
             eigen_to_std_vector<bopt::variable>::convert(
                 contact.parameters_v.friction_force_upper_bound)});
    }

    std::shared_ptr<ContactPoint3D> get_contact_point_3d(const string_t &name) {
        if (contacts_.contact_3d_.find(name) != contacts_.contact_3d_.end()) {
            return contacts_.contact_3d_.at(name);
        }
        return nullptr;
    }

    void add_holonomic_constraint_to_program(
        const HolonomicConstraint &constraint) {
        // Create new variables
        eigen_vector_var_t lambda =
            create_variable_vector("lambda", constraint.dimension());

        variables.lambda.conservativeResize(variables.lambda.size() +
                                            lambda.size());
        variables.lambda.bottomRows(lambda.size()) = lambda;

        // Add lamba to variables
        for (std::size_t i = 0; i < lambda.size(); ++i) {
            program.add_variable(lambda[i]);
        }
    }

    void add_dynamics_to_program(Dynamics &dynamics) {
        auto dynamics_constraint = dynamics.to_constraint();

        // Bind to program
        program.add_linear_constraint(
            dynamics_constraint,
            // Variables
            {eigen_to_std_vector<bopt::variable>::convert(variables.a),
             eigen_to_std_vector<bopt::variable>::convert(variables.u),
             eigen_to_std_vector<bopt::variable>::convert(variables.lambda)},
            // contact-specific parameters
            {eigen_to_std_vector<bopt::variable>::convert(parameters.q),
             eigen_to_std_vector<bopt::variable>::convert(parameters.v)});
    }

    bopt::mathematical_program<double> program;

    osc_variables<eigen_vector_var_t> variables;
    osc_parameters<eigen_vector_var_t> parameters;

    template <class TaskType>
    void update_task(const model_state_t &model_state, TaskType &task) {
        // Set weighting
        // program.set_parameter(task.parameters().w, task.parameters().w);
        typedef typename task_traits<TaskType>::task_state_t task_state_t;
        typedef typename task_traits<TaskType>::task_error_t task_error_t;
        typedef typename task_traits<TaskType>::reference_t reference_t;

        task_state_t task_state;
        // Evaluate task error
        task.get_task_state(model_state, task_state);
        task_error_t task_error(task.dimension());
        task.get_task_error(task_state, task_error);

        // Set desired acceleration as PID output
        task.parameters().desired_task_acceleration =
            task.pid.compute(task_error);

        for (std::size_t i = 0; i < task.parameters().w.size(); ++i) {
            program.set_parameter(task.parameters_v.w[i],
                                  task.parameters().w[i]);
        }
    }

    template <class ContactPoint>
    void update_contact_point(const model_state_t &model_state,
                              ContactPoint &contact) {
        // todo - detect a change in contact to minimise variable setting
        if (contact.in_contact) {
            for (std::size_t i = 0; i < contact.dimension(); ++i) {
                // program.set_parameter(
                //     contact.parameters_v.friction_force_upper_bound[i],
                //     contact.parameters().friction_force_upper_bound[i]);

                // program.set_parameter(
                //     contact.parameters_v.friction_force_lower_bound[i],
                //     contact.parameters().friction_force_lower_bound[i]);
            }

            for (std::size_t i = 0; i < 3; ++i) {
                program.set_parameter(contact.parameters_v.n[i],
                                      contact.parameters().n[i]);
                program.set_parameter(contact.parameters_v.t[i],
                                      contact.parameters().t[i]);
                program.set_parameter(contact.parameters_v.b[i],
                                      contact.parameters().b[i]);
            }
            program.set_parameter(contact.parameters_v.mu,
                                  contact.parameters().mu);
            // contact.parameters().friction_force_lower_bound =
            // contact.parameters().friction_force_upper_bound =
        } else {
        }
    }

   private:
    // Task map
    osc_task_maps tasks_;
    // Contact map
    osc_contact_point_maps contacts_;

    // System dynamics
    Dynamics dynamics_;

    // Symbolic model
    model_sym_t &model;

    // Initialisation flag
    bool is_initialised_ = false;

    // QP solver
    std::unique_ptr<bopt::solvers::qpoases_solver_instance> qp_;
};

}  // namespace osc
