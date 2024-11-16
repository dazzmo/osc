#include "osc/task.hpp"

#include <bopt/ad/casadi/casadi.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

PositionTask::PositionTask(const model_sym_t &model, const std::string &target,
                           const std::string &reference_frame)
    : Task(3, model.nq, model.nv), target_frame(target), reference_frame(reference_frame) {
    eigen_vector_sym_t q = create_symbolic_vector("q", model_nq());
    eigen_vector_sym_t v = create_symbolic_vector("v", model_nv());
    eigen_vector_sym_t a(model_nv());
    a.setZero();

    // Compute frame state
    frame_state<sym_t> frame =
        get_frame_state(model, q, v, a, target, reference_frame);

    // Create expression evaluators
    sym_t q_s = eigen_to_casadi<sym_elem_t>::convert(q);
    sym_t v_s = eigen_to_casadi<sym_elem_t>::convert(v);

    sym_t xpos_s =
        eigen_to_casadi<sym_elem_t>::convert(frame.pos.translation());
    sym_t xvel_s = eigen_to_casadi<sym_elem_t>::convert(frame.vel.linear());

    xpos = std::make_unique<expression_evaluator_t>(xpos_s,
                                                    sym_vector_t({q_s, v_s}));
    xvel = std::make_unique<expression_evaluator_t>(xvel_s,
                                                    sym_vector_t({q_s, v_s}));
}

bopt::quadratic_cost<PositionTask::value_type>::shared_ptr
PositionTask::to_task_cost(const model_sym_t &model) const {
    eigen_vector_sym_t q = create_symbolic_vector("q", model_nq());
    eigen_vector_sym_t v = create_symbolic_vector("v", model_nv());
    eigen_vector_sym_t a = create_symbolic_vector("a", model_nv());

    eigen_vector_sym_t w = create_symbolic_vector("w", 3);
    eigen_vector_sym_t xacc_d = create_symbolic_vector("xacc_d", 3);

    // Compute frame state
    frame_state<sym_t> frame =
        get_frame_state(model, q, v, a, target_frame, reference_frame);

    eigen_vector_sym_t dxacc = frame.acc.linear() - xacc_d;

    sym_t q_s = eigen_to_casadi<sym_elem_t>::convert(q);
    sym_t v_s = eigen_to_casadi<sym_elem_t>::convert(v);
    sym_t a_s = eigen_to_casadi<sym_elem_t>::convert(a);
    sym_t w_s = eigen_to_casadi<sym_elem_t>::convert(w);
    sym_t xacc_d_s = eigen_to_casadi<sym_elem_t>::convert(xacc_d);

    // Compute weighted squared norm
    sym_t cost = dxacc.transpose() * w.asDiagonal() * dxacc;

    return bopt::casadi::quadratic_cost<value_type>::create(
        cost, a_s, sym_vector_t({q_s, v_s, w_s, xacc_d_s}));
}

// CentreOfMassTask::CentreOfMassTask(const model_sym_t &model,
//                                    const std::string &target,
//                                    const std::string &reference_frame) {
//     eigen_vector_sym_t q;
//     eigen_vector_sym_t v;
//     eigen_vector_sym_t a;

//     pinocchio::DataTpl<sym_t> data(model);

//     // Compute frame state
//     pinocchio::forwardKinematics(model, data, q, v, a);
//     pinocchio::updateFramePlacements(model, data);

//     pinocchio::centerOfMass(model, data, q, v, a, false);
//     // Get data for centre of mass
//     sym_vector_t com = data.com[0], com_vel = data.vcom[0], com_acc =
//     data.acom[0];

//     // todo - convert to local frame

//     // Create expression evaluators
//     sym_t q_s = eigen_to_casadi<sym_t>::convert(q);
//     sym_t v_s = eigen_to_casadi<sym_t>::convert(v);
//     sym_t a_s = eigen_to_casadi<sym_t>::convert(a);

//     sym_t com_s =
//     eigen_to_casadi<sym_t>::convert(frame.pos.translation()); sym_t
//     com_vel_s = eigen_to_casadi<sym_t>::convert(com_vel); sym_t com_acc_s
//     = eigen_to_casadi<sym_t>::convert(com_acc);

//     xpos = std::make_unique<expression_evaluator_t>(com_s, {q_s, v_s}, {});
//     xvel = std::make_unique<expression_evaluator_t>(com_vel_s, {q_s, v_s},
//     {}); xacc = std::make_unique<expression_evaluator_t>(com_acc_s, {q_s,
//     v_s}, {});
// }

// OrientationTask::OrientationTask(const model_sym_t &model,
//                                  const std::string &target,
//                                  const std::string &reference_frame) {
//     sym_vector_t q;
//     sym_vector_t v;
//     sym_vector_t a;

//     // Compute frame state
//     frame_state<sym_t> frame =
//         get_frame_state(model, q, v, a, target, reference_frame);

//     // Create expression evaluators
//     sym_t q_s = eigen_to_casadi<sym_t>::convert(q);
//     sym_t v_s = eigen_to_casadi<sym_t>::convert(v);
//     sym_t a_s = eigen_to_casadi<sym_t>::convert(a);

//     sym_t xpos_s = eigen_to_casadi<sym_t>::convert(
//         frame.pos.rotation());
//     sym_t xvel_s = eigen_to_casadi<sym_t>::convert(frame.vel.angular());
//     sym_t xacc_s = eigen_to_casadi<sym_t>::convert(frame.acc.angular());

//     xpos = std::make_unique<expression_evaluator_t>(xpos_s, {q_s, v_s}, {});
//     xvel = std::make_unique<expression_evaluator_t>(xvel_s, {q_s, v_s}, {});
//     xacc = std::make_unique<expression_evaluator_t>(xacc_s, {q_s, v_s}, {});
// }

// SE3Task::SE3Task(const model_sym_t &model, const std::string &target,
//                  const std::string &reference_frame) {
//     sym_vector_t q;
//     sym_vector_t v;
//     sym_vector_t a;

//     // Compute frame state
//     frame_state<sym_t> frame =
//         get_frame_state(model, q, v, a, target, reference_frame);

//     // Create expression evaluators
//     sym_t q_s = eigen_to_casadi<sym_t>::convert(q);
//     sym_t v_s = eigen_to_casadi<sym_t>::convert(v);
//     sym_t a_s = eigen_to_casadi<sym_t>::convert(a);

//     // sym_t xpos_s = eigen_to_casadi<sym_t>::convert(frame.pos.to()); //
//     // todo - make this a 7d vector
//     sym_t xvel_s = eigen_to_casadi<sym_t>::convert(frame.vel.toVector());
//     sym_t xacc_s = eigen_to_casadi<sym_t>::convert(frame.acc.toVector());

//     xpos = std::make_unique<expression_evaluator_t>(xpos_s, {q_s, v_s}, {});
//     xvel = std::make_unique<expression_evaluator_t>(xvel_s, {q_s, v_s}, {});
//     xacc = std::make_unique<expression_evaluator_t>(xacc_s, {q_s, v_s}, {});
// }

// integer_type SE3Task::get_error(const task_state &task_state,
//                                 task_error &error) {
//     // Compute difference through logarithmic map
//     pinocchio::SE3 p;
//     pinocchio::log6(p);

//     error.positional =  - reference.position;
//     error.derivative = task_state.velocity - reference.velocity;
//     return integer_type(0);
// }

}  // namespace osc