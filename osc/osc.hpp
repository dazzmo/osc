#pragma once

#include <bopt/program.h>

#include "osc/contact.hpp"
#include "osc/task.hpp"

namespace osc {

/**
 * @brief Visitor class that can be customised and added to a typical OSC
 * instance
 *
 */
class osc_visitor {
    void init();
    void update_references();
    void update_parameters();
};

struct osc_task_maps {
    template <typename TaskType>
    using task_map_t = std::unordered_map<string_t, std::shared_ptr<TaskType>>;

    task_map_t<PositionTask> position_tasks_;
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
    OSC(model_sym_t &model) : model(model) {
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
        // dynamics_->add_constraint(program);
        // visitor.init();
    }

    /**
     * @brief Loop function that computes the optimal control by solving an OCP
     *
     */
    void loop();

    void add_position_task(const std::string &name,
                           std::shared_ptr<PositionTask> &task) {
        tasks_.position_tasks_.insert({name, task});
        add_to_program(*task);
    }

    template <class TaskType>

    void add_to_program(const TaskType &task) {
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

    // void add_position_task(const string_t &name,
    //                        const PositionTask::shared_ptr &task);
    // void add_orientation_task(const string_t &name,
    //                        const OrientationTask::shared_ptr &task);
    // void add_se3_task(const string_t &name,
    //                        const SE3Task::shared_ptr &task);

    // void get_position_task(const string_t &task);
    // void get_orientation_task(const string_t &task);
    // void get_se3_task(const string_t &task);

    // void add_contact_point(const ContactTask &point) {
    //     dynamics_->register_contact_point(point);

    //     // point.add_to_program(program);

    //     contacts_.push_back(point);
    // }

    void get_contact_point(const string_t &name) {}

    bopt::mathematical_program<double> program;

    osc_variables<eigen_vector_var_t> variables;
    osc_parameters<eigen_vector_var_t> parameters;

    // Solver instance
    // bopt::solvers::qpoases_solver_instance qp_solver;

    template <class TaskType>
    void update_task(TaskType &task) {
        // Set weighting
        // program.set_parameter(task.parameters().w, task.parameters().w);
        typedef typename task_traits<TaskType>::task_state_t task_state_t;
        typedef typename task_traits<TaskType>::task_error_t task_error_t;
        typedef typename task_traits<TaskType>::reference_t reference_t;

        Task::model_state_t model_state(1, 1);

        task_state_t task_state;
        // Evaluate task error
        task.get_task_state(model_state, task_state);
        task_error_t task_error(task.dimension());
        task.get_task_error(task_state, task_error);

        // Set desired acceleration as PID output
        task.parameters().desired_task_acceleration =
            task.pid.compute(task_error);
        // program.set_parameter(task.parameters().desired_task_acceleration,
        // task.parameters().q);
    }

   private:
    osc_task_maps tasks_;

    model_sym_t &model;

    // task_map_t<OrientationTask> orientation_tasks_;
    // task_map_t<SE3Task> se3_tasks_;

    std::vector<Task> get_all_tasks() {
        std::vector<Task> all;
        // all.assign(all.end(), position_tasks_.begin(),
        // position_tasks_.end()); all.assign(all.end(),
        // orientation_tasks_.begin(),
        //            orientation_tasks_.end());
        // all.assign(all.end(), se3_tasks_.begin(), se3_tasks_.end());

        return all;
    }

    // std::unique_ptr<Dynamics> dynamics_;
};

}  // namespace osc
