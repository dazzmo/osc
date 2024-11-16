#include <bopt/program.h>

#include "osc/contact.hpp"
#include "osc/task.hpp"

namespace osc {

class OSC {
    OSC(model_sym_t &model) {
        // Create system dynamics
    }

    void init() {
        // All tasks added, finalise dynamics constraint
        dynamics_->add_constraint(program);
    }

    /**
     * @brief Loop function that computes the optimal control by solving an OCP
     *
     */
    void loop();

    void add_task(const std::shared_ptr<Task> &task) {
        task->add_to_program(program);
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



    void add_contact_point(const ContactTask &point) {
        dynamics_->register_contact_point(point);

        // point.add_to_program(program);

        contacts_.push_back(point);
    }

    void get_contact_point(const string_t &name) {

    }

    bopt::mathematical_program<double> program;

    // Solver instance
    // bopt::solvers::qpoases_solver_instance qp_solver;

   private:
    typedef std::string string_t;
    template <typename TaskType>
    using task_map_t = std::unordered_map<string_t, TaskType>;

    task_map_t<PositionTask> position_tasks_;
    task_map_t<OrientationTask> orientation_tasks_;
    task_map_t<SE3Task> se3_tasks_;

    std::vector<Task> get_all_tasks() {
        std::vector<Task> all;
        all.assign(all.end(), position_tasks_.begin(), position_tasks_.end());
        all.assign(all.end(), orientation_tasks_.begin(),
                   orientation_tasks_.end());
        all.assign(all.end(), se3_tasks_.begin(), se3_tasks_.end());

        return all;
    }

    std::unique_ptr<Dynamics> dynamics_;
};

}  // namespace osc
