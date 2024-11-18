#include "osc/osc.hpp"

namespace osc {

void OSC::loop(const state<eigen_vector_t> &model_state) {
    assert(is_initialised_ && "OSC is not initialised!");
    // Update references for all tasks
    for (std::size_t i = 0; i < model_state.position.size(); ++i) {
        program.set_parameter(parameters.q[i], model_state.position[i]);
    }
    for (std::size_t i = 0; i < model_state.velocity.size(); ++i) {
        program.set_parameter(parameters.v[i], model_state.velocity[i]);
    }

    for (auto &task : tasks_.position_tasks_) {
        update_task(model_state, *task.second);
    }

    for (auto &task : tasks_.orientation_tasks_) {
        update_task(model_state, *task.second);
    }

    for (auto &task : tasks_.se3_tasks_) {
        update_task(model_state, *task.second);
    }

    for (auto &contact : contacts_.contact_3d_) {
        update_contact_point(model_state, *contact.second);
    }

    // Once updated, solve the program
    qp_->solve();

    // if(reset) qp.reset()
}

}  // namespace osc