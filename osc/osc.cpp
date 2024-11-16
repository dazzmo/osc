#include "osc/osc.hpp"

namespace osc {

void OSC::init() {
    // Create data
    // Create ...
}

void OSC::loop() {
    // Update references for all tasks
    model_state state;
    // program.set_parameter(q, q);
    // program.set_parameter(v, v);

    for (auto &task : get_all_tasks()) {
        // Update program parameters

        // Set weighting
        // program.set_parameter(task.parameters().w, task.parameters().w);
        task_state task_state;
        // Evaluate task error
        task.get_state(state, task_state);
        task_error task_error;
        task.get_error(task_state, task_error);

        // Set desired acceleration as PID output
        task.parameters().desired_task_acceleration =
            task.pid.compute(task_error);
        // program.set_parameter(task.parameters().desired_task_acceleration,
        // task.parameters().q);
    }

    for (auto &contact : contacts_) {
        if (contact.in_contact) {
            // contact.parameters().friction_force_lower_bound =
            // contact.parameters().friction_force_upper_bound =
        } else {
            contact.parameters().friction_force_lower_bound.assign(3, 0);
            contact.parameters().friction_force_upper_bound.assign(3, 0);
        }
    }

    // Once updated, solve the program
    // qp.solve()

    // if(reset) qp.reset()
}

}  // namespace osc