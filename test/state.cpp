#include <pinocchio/parsers/urdf.hpp>

#include "osc/OSC.hpp"
#include "osc/limits/Velocity.hpp"
#include "osc/tasks/CentreOfMass.hpp"
#include "osc/tasks/Damping.hpp"
#include "osc/tasks/Frame.hpp"
#include "osc/tasks/Posture.hpp"
#include "osc/contacts/Contact3D.hpp"
#include "osc/FrictionConeModel.hpp"
#include "osc/Problem.hpp"

int main(void) {
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(),
                                model);

    // Create a state
    osc::State state(model);

    auto q0 = pinocchio::neutral(model);
    auto v0 = osc::Vector::Random(model.nv);

    state.update(q0, v0);

    // Create task
    auto frame = std::make_shared<osc::FrameTask>("LeftFootPitch");
    auto posture = std::make_shared<osc::PostureTask>(state);
    posture->setTarget(q0);
    auto com = std::make_shared<osc::CentreOfMassTask>();
    auto damping = std::make_shared<osc::DampingTask>(state);
    auto limit = std::make_shared<osc::VelocityLimit>(state);


    auto friction_model = std::make_shared<osc::LinearisedFrictionConeModel>(4);
    auto contact = std::make_shared<osc::Contact3D>("RightFootPitch", friction_model);


    auto problem = osc::OSCProgram(state, 10);
    problem.addMotionTask(frame, 0.0, 1.0);
    problem.addMotionTask(posture, 0.0, 0.1);
    problem.addMotionTask(com, 0.0, 0.1);
    problem.addMotionTask(damping, 0.0, 0.1);
    problem.addMotionLimit(limit, 0.0, 0.1);
    problem.addContact(contact, 0.0, 0.01);

    problem.solve(0.0, state, 0.1);

    return 0;
}