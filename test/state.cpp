#include <chrono>
#include <pinocchio/parsers/urdf.hpp>

#include "osc/FrictionConeModel.hpp"
#include "osc/OSC.hpp"
#include "osc/Problem.hpp"
#include "osc/contacts/Contact3D.hpp"
#include "osc/limits/Configuration.hpp"
#include "osc/limits/Velocity.hpp"
#include "osc/tasks/CentreOfMass.hpp"
#include "osc/tasks/Damping.hpp"
#include "osc/tasks/Frame.hpp"
#include "osc/tasks/Posture.hpp"

class MyActuation : public osc::ActuationAbstract {
   public:
    MyActuation(const pinocchio::Model &model)
        : osc::ActuationAbstract(10, model.nv) {}

    osc::Vector compute(const osc::State &state,
                        const Eigen::Ref<const osc::Vector> &u) const override {
        return osc::Vector::Zero(10);
    }

    void computeJacobian(const osc::State &state,
                         Eigen::Ref<osc::Matrix> jacobian) const override {
        jacobian(7, 0) = 1.0;
        jacobian(8, 1) = 1.0;
        jacobian(9, 2) = 1.0;
        jacobian(10, 3) = 1.0;
        jacobian(14, 4) = 1.0;

        jacobian(15, 5) = 1.0;
        jacobian(16, 6) = 1.0;
        jacobian(17, 7) = 1.0;
        jacobian(18, 8) = 1.0;
        jacobian(21, 9) = 1.0;
    }
};

int main(void) {
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(),
                                model);

    // Create actuation
    auto u = std::make_shared<MyActuation>(model);

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
    auto limit = std::make_shared<osc::ConfigurationLimit>(state);

    auto friction_model = std::make_shared<osc::LinearisedFrictionConeModel>(4);
    auto contact =
        std::make_shared<osc::Contact3D>("RightFootPitch", friction_model);

    auto problem = osc::OSCProgram(state, 10);
    problem.addActuation(u);
    problem.addMotionTask(frame, 0.0, 1.0);
    problem.addMotionTask(posture, 0.0, 0.1);
    problem.addMotionTask(com, 0.0, 0.1);
    problem.addMotionTask(damping, 0.0, 0.1);
    problem.addContact(contact, 0.0, 0.01);

    bool remove = false;
    for (double t = 0.0; t <= 1.1; t += 0.01) {
        if (t > 0.4 && !remove) {
            problem.removeContact(contact, t, 0.2);
            remove = true;
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        problem.solve(t, state, 0.1);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> d = t2 - t1;
        std::cout << d.count() << std::endl;
        std::cout << "f = " << problem.getContactForce(contact).transpose() << std::endl;
    }

    return 0;
}