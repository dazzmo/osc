#include "osc/osc.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

void OSC::loop(const eigen_vector_t &q, const eigen_vector_t &v) {
    assert(is_initialised_ && "OSC is not initialised!");
    // Update references for all tasks
    for (std::size_t i = 0; i < q.size(); ++i) {
        program.set_parameter(parameters.q[i], q[i]);
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
        program.set_parameter(parameters.v[i], v[i]);
    }

    data_t data(model);

    // Update pinocchio model for computations
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::centerOfMass(model, data, q, v);
    pinocchio::updateFramePlacements(model, data);

    for (auto &task : get_all_tasks()) {
        update_task(model, data, task);
    }

    for (auto &contact : get_all_contact_points()) {
        update_contact_point(model, data, contact);
    }

    // Once updated, solve the program
    qp_->solve();

    // if(reset) qp.reset()
}

}  // namespace osc