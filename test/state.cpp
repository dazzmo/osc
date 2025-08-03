#include "osc/OSC.hpp"
#include "osc/tasks/Frame.hpp"
#include "osc/tasks/Posture.hpp"
#include "osc/tasks/Damping.hpp"
#include "osc/tasks/CentreOfMass.hpp"

#include <pinocchio/parsers/urdf.hpp>

int main(void) {
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);

    // Create a state
    osc::State state(model);

    auto q0 = pinocchio::neutral(model);
    auto v0 = osc::Vector::Random(model.nv);

    state.update(q0, v0);

    // Create task
    // auto task = std::make_shared<osc::FrameTask>("LeftFootPitch");
    // auto task = std::make_shared<osc::PostureTask>(state);
    // task->setTarget(q0);
    // auto task = std::make_shared<osc::CentreOfMassTask>();
    auto task = std::make_shared<osc::DampingTask>(state);

    osc::Vector e(task->getDimension()), dot_e(task->getDimension());
    osc::Matrix jac(task->getDimension(), state.nv());
    task->computeError(state, e, dot_e);
    task->computeJacobian(state, jac);
    std::cout << jac << std::endl;
    task->computeAccelerationBias(state, e);
    std::cout << e << std::endl;

    return 0;
}