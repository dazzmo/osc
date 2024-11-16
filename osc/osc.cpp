#include "osc/osc.hpp"

namespace osc {


void OSC::loop() {
    // Update references for all tasks
    // program.set_parameter(q, q);
    // program.set_parameter(v, v);

    for (auto &task : tasks_.position_tasks_) {
        update_task(*task.second);
    }

    // visitor.loop();

    // for (auto &contact : contacts_) {
    //     if (contact.in_contact) {
    //         // contact.parameters().friction_force_lower_bound =
    //         // contact.parameters().friction_force_upper_bound =
    //     } else {
    //         contact.parameters().friction_force_lower_bound.assign(3, 0);
    //         contact.parameters().friction_force_upper_bound.assign(3, 0);
    //     }
    // }

    // Once updated, solve the program
    // qp.solve()

    // if(reset) qp.reset()
}

}  // namespace osc