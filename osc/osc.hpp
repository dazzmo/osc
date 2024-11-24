#pragma once

#include <bopt/program.hpp>
#include <bopt/solvers/qpoases.hpp>

#include "osc/contact.hpp"
#include "osc/dynamics.hpp"
#include "osc/task.hpp"
#include "osc/constraint.hpp"
#include "osc/cost.hpp"

namespace osc {

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
    OSC(model_t &model) : model(model), model_sym(model.cast<sym_t>()) {
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
        qp_ = std::make_unique<bopt::solvers::qpoases_solver_instance>(program);

        // Create solver
        is_initialised_ = true;
    }

    /**
     * @brief Loop function that computes the optimal control by solving an OCP
     *
     */
    void loop(const eigen_vector_t &q, const eigen_vector_t &v);

    /**
     * @brief Add a frame task to the program
     *
     * @param name
     * @param task
     */
    void add_frame_task(const std::string &name,
                        std::shared_ptr<FrameTask> &task) {
        frame_tasks_index_map_.insert({name, frame_tasks_.size()});
        frame_tasks_.push_back(task);
        add_task_to_program(*task);
    }

    std::shared_ptr<FrameTask> get_frame_task(const string_t &name) {
        if (frame_tasks_index_map_.find(name) != frame_tasks_index_map_.end()) {
            return frame_tasks_[frame_tasks_index_map_.at(name)];
        }
        return nullptr;
    }

    void add_joint_tracking_task(const std::string &name,
                                 std::shared_ptr<JointTrackingTask> &task) {
        joint_tracking_tasks_index_map_.insert(
            {name, joint_tracking_tasks_.size()});
        joint_tracking_tasks_.push_back(task);
        add_task_to_program(*task);
    }

    std::shared_ptr<JointTrackingTask> get_joint_tracking_task(
        const string_t &name) {
        if (joint_tracking_tasks_index_map_.find(name) !=
            joint_tracking_tasks_index_map_.end()) {
            return joint_tracking_tasks_[joint_tracking_tasks_index_map_.at(
                name)];
        }
        return nullptr;
    }

    void add_centre_of_mass_task(std::shared_ptr<CentreOfMassTask> &task) {
        com_task_ = task;
        add_task_to_program(*task);
    }

    std::shared_ptr<CentreOfMassTask> get_centre_of_mass_task() {
        return com_task_;
    }

    template <class CostType>
    void add_cost_to_program(const CostType &cost) {
        cost->add_to_program(model_sym, *this);
    }

    void add_contact_point_3d(const std::string &name,
                              std::shared_ptr<ContactPoint3D> &contact,
                              std::shared_ptr<Dynamics> &dynamics) {
        contact_3d_index_map_.insert({name, contact_3d_.size()});
        contact_3d_.push_back(contact);

        add_constraint_to_program(*contact, dynamics);
    }

    std::shared_ptr<ContactPoint3D> get_contact_point_3d(const string_t &name) {
        if (contact_3d_index_map_.find(name) != contact_3d_index_map_.end()) {
            return contact_3d_[contact_3d_index_map_.at(name)];
        }
        return nullptr;
    }

    void add_dynamics(std::shared_ptr<Dynamics> &dynamics) {
        dynamics->add_to_program(model_sym, *this);
    }

    void add_constraint_to_program(HolonomicConstraint &constraint,
                                   std::shared_ptr<Dynamics> &dynamics) {
        // Create new variables
        add_constraint_forces(
            create_variable_vector("lambda", constraint.dimension()));

        // Add constraint to dynamics
        dynamics->add_constraint(constraint);
        // Add constraint to program
        constraint.add_to_program(model_sym, *this);
    }

    bopt::mathematical_program<double> program;

    osc_variables<eigen_vector_var_t> variables;
    osc_parameters<eigen_vector_var_t> parameters;

    std::vector<std::shared_ptr<Task>> get_all_tasks() {
        std::vector<std::shared_ptr<Task>> tasks = {};
        tasks.insert(tasks.end(), frame_tasks_.begin(), frame_tasks_.end());
        tasks.insert(tasks.end(), joint_tracking_tasks_.begin(),
                     joint_tracking_tasks_.end());
        if (com_task_) tasks.push_back(com_task_);
        return tasks;
    }

    std::vector<std::shared_ptr<ContactPoint>> get_all_contact_points() {
        std::vector<std::shared_ptr<ContactPoint>> contacts = {};
        contacts.insert(contacts.end(), contact_3d_.begin(), contact_3d_.end());
        return contacts;
    }

   private:
    // Tasks
    std::unordered_map<std::string, std::size_t> frame_tasks_index_map_;
    std::unordered_map<std::string, std::size_t>
        joint_tracking_tasks_index_map_;

    std::vector<std::shared_ptr<FrameTask>> frame_tasks_;
    std::vector<std::shared_ptr<JointTrackingTask>> joint_tracking_tasks_;
    std::shared_ptr<CentreOfMassTask> com_task_ = nullptr;

    // Contacts
    std::unordered_map<std::string, std::size_t> contact_3d_index_map_;

    std::vector<std::shared_ptr<ContactPoint3D>> contact_3d_;

    // Costs
    std::unordered_map<std::string, std::size_t> costs_index_map_;
    std::vector<std::shared_ptr<Cost>> costs_;

    template <class TaskType>
    void add_task_to_program(TaskType &task) {
        task.add_to_program(model_sym, *this);
    }

    template <class TaskType>
    void update_task(const model_t &model, const data_t &data, TaskType &task) {
        task_error<eigen_vector_t> e(task->dimension());
        task->compute_task_error(model, data, e);

        // Compute desired acceleration
        task->parameters().xacc_d = task->pid.compute(e);

        // Update parameters
        for (std::size_t i = 0; i < task->dimension(); ++i) {
            program.set_parameter(task->parameters_v.w[i],
                                  task->parameters_d.w[i]);
            program.set_parameter(task->parameters_v.xacc_d[i],
                                  task->parameters_d.xacc_d[i]);
        }
    }

    template <class ContactPoint>
    void update_contact_point(const model_t &model, const data_t &data,
                              ContactPoint &contact) {
        // todo - detect a change in contact to minimise variable setting
        if (contact->in_contact) {
            for (std::size_t i = 0; i < contact->dimension(); ++i) {
                program.set_parameter(contact->parameters_v.lambda_ub[i],
                                      contact->parameters().lambda_ub[i]);

                program.set_parameter(contact->parameters_v.lambda_lb[i],
                                      contact->parameters().lambda_lb[i]);
            }

            for (std::size_t i = 0; i < 3; ++i) {
                program.set_parameter(contact->parameters_v.n[i],
                                      contact->parameters().n[i]);
                program.set_parameter(contact->parameters_v.t[i],
                                      contact->parameters().t[i]);
                program.set_parameter(contact->parameters_v.b[i],
                                      contact->parameters().b[i]);
            }
            program.set_parameter(contact->parameters_v.mu,
                                  contact->parameters().mu);
        } else {
            for (std::size_t i = 0; i < contact->dimension(); ++i) {
                program.set_parameter(contact->parameters_v.lambda_ub[i], 0.0);

                program.set_parameter(contact->parameters_v.lambda_lb[i], 0.0);
            }
        }
    }

    /**
     * @brief Adds force variables to the program
     *
     * @param lambda
     */
    void add_constraint_forces(const eigen_vector_var_t &lambda) {
        variables.lambda.conservativeResize(variables.lambda.size() +
                                            lambda.size());
        variables.lambda.bottomRows(lambda.size()) << lambda;

        // Add lamba to variables
        for (std::size_t i = 0; i < lambda.size(); ++i) {
            program.add_variable(lambda[i]);
        }
    }

    // Model
    model_t model;
    // Symbolic model
    model_sym_t model_sym;

    // Initialisation flag
    bool is_initialised_ = false;

    // QP solver
    std::unique_ptr<bopt::solvers::qpoases_solver_instance> qp_;
};

}  // namespace osc
