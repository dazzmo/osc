#include "osc/osc.hpp"

namespace osc {

void OSC::loop() {
    // Update references for all tasks
    // program.set_parameter(q, q);
    // program.set_parameter(v, v);

    for (auto &task : tasks_.position_tasks_) {
        update_task(*task.second);
    }

    for (auto &task : tasks_.orientation_tasks_) {
        update_task(*task.second);
    }

    for (auto &task : tasks_.se3_tasks_) {
        update_task(*task.second);
    }

    // visitor.loop();

    for (auto &contact : contacts_.contact_3d_) {
        // todo - detect a change in contact to minimise variable setting
        if (contact.second->in_contact) {
            for (std::size_t i = 0; i < contact.second->dimension(); ++i) {
                // program.set_parameter(
                //     contact.second->parameters_v.friction_force_upper_bound[i],
                //     contact.second->parameters().friction_force_upper_bound[i]);

                // program.set_parameter(
                //     contact.second->parameters_v.friction_force_lower_bound[i],
                //     contact.second->parameters().friction_force_lower_bound[i]);
            }

            for (std::size_t i = 0; i < 3; ++i) {
                program.set_parameter(contact.second->parameters_v.n[i],
                                      contact.second->parameters().n[i]);
                program.set_parameter(contact.second->parameters_v.t[i],
                                      contact.second->parameters().t[i]);
                program.set_parameter(contact.second->parameters_v.b[i],
                                      contact.second->parameters().b[i]);
            }
            program.set_parameter(contact.second->parameters_v.mu,
                                  contact.second->parameters().mu);
            // contact.second->parameters().friction_force_lower_bound =
            // contact.second->parameters().friction_force_upper_bound =
        } else {
        }
    }

    // Once updated, solve the program
    // qp.solve()

    // if(reset) qp.reset()
}

}  // namespace osc