#include "osc/osc.hpp"

namespace osc {

void OSC::loop() {
    assert(is_initialised_ && "OSC is not initialised!");
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

    for (auto &contact : contacts_.contact_3d_) {
        update_contact_point(*contact.second);
    }

    // Once updated, solve the program
    qp_->solve();

    // if(reset) qp.reset()
}

}  // namespace osc