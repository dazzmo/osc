#include "osc/task.hpp"

#include <bopt/ad/casadi/casadi.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

bopt::quadratic_cost<PositionTask::value_type>::shared_ptr
PositionTask::to_task_cost() {
    // Convert parameters to useable vectors
    typedef Eigen::VectorX<sym_t> sym_vector_t;
    typedef pinocchio::SE3Tpl<sym_t> se3_t;
    typedef pinocchio::MotionTpl<sym_t> motion_t;

    sym_vector_t q;
    sym_vector_t v;
    sym_vector_t a;

    sym_vector_t xacc_a;
    sym_vector_t xacc_d;

    sym_vector_t w;

    model_sym_t model;
    data_sym_t data;

    // Compute the task
    pinocchio::forwardKinematics(model, data, q, v, a);
    pinocchio::updateFramePlacements(model, data);

    // Compute the acceleration of the frame
    std::string frame = "root";
    std::string reference = "root";
    motion_t xacc_a_o = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame), pinocchio::WORLD);

    // Get frame of system
    se3_t oMb = model.frames[model.getFrameId(reference)];

    // Convert to desired reference frame
    motion_t xacc_a_b = oMb.actInv(xacc_a_o);

    sym_vector_t diff = xacc_a_b.toVector() - xacc_d;
    
    sym_t cost = (diff).dot(diff);

    return bopt::casadi::quadratic_cost<value_type>::create(cost, a, q);
}

void PositionTask::add_task_cost(
    bopt::mathematical_program<value_type> &program) {
    auto cost = to_task_cost();

    // Bind to program
    program.add_quadratic_cost(cost, {{}},
                               {{parameters.q.data(), parameters.v.data(),
                                 parameters.desired_task_acceleration.data()}});
}

integer_type PositionTask::get_state(const model_state &state,
                                     task_state &task_state) {
    // Evaluate functions
    x_pos({state.position.data(), state.velocity.data()},
          {task_state.position.data()});
    x_vel({state.position.data(), state.velocity.data()},
          {task_state.velocity.data()});

    return integer_type(0);
}

integer_type PositionTask::get_error(const task_state &task_state,
                                     task_error &error) {
    error.positional = task_state.position - reference.position;
    error.derivative = task_state.velocity - reference.velocity;
    return integer_type(0);
}

}  // namespace osc