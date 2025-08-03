#include "osc/OSC.hpp"

#include <pinocchio/parsers/urdf.hpp>

int main(void) {
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    // Create a state
    osc::State state(model);

    auto q0 = pinocchio::neutral(model);
    auto v0 = osc::Vector::Zero(model.nv);

    state.update(q0, v0);
    return 0;
}