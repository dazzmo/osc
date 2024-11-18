#include "osc/task.hpp"

#include <bopt/ad/casadi/casadi.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

PositionTask::PositionTask(const model_sym_t &model, const std::string &target,
                           const std::string &reference_frame)
    : Task(3, model.nq, model.nv),
      target_frame(target),
      reference_frame(reference_frame) {
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

    eigen_vector_sym_t w = create_symbolic_vector("w", dimension());
    eigen_vector_sym_t xacc_d = create_symbolic_vector("xacc_d", dimension());

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

    // Concatenate parameters
    sym_t p = sym_t::vertcat({q_s, v_s, w_s, xacc_d_s});

    return bopt::casadi::quadratic_cost<value_type>::create(cost, a_s,
                                                            sym_vector_t({p}));
}

OrientationTask::OrientationTask(const model_sym_t &model,
                                 const std::string &target,
                                 const std::string &reference_frame)
    : Task(3, model.nq, model.nv),
      target_frame(target),
      reference_frame(reference_frame) {
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

    // Flatten rotation matrix
    eigen_vector_sym_t rotation_vec(Eigen::Map<eigen_vector_sym_t>(
        frame.pos.rotation().data(),
        frame.pos.rotation().cols() * frame.pos.rotation().rows()));

    sym_t xpos_s = eigen_to_casadi<sym_elem_t>::convert(rotation_vec);
    sym_t xvel_s = eigen_to_casadi<sym_elem_t>::convert(frame.vel.linear());

    xpos = std::make_unique<expression_evaluator_t>(xpos_s,
                                                    sym_vector_t({q_s, v_s}));
    xvel = std::make_unique<expression_evaluator_t>(xvel_s,
                                                    sym_vector_t({q_s, v_s}));
}

bopt::quadratic_cost<OrientationTask::value_type>::shared_ptr
OrientationTask::to_task_cost(const model_sym_t &model) const {
    eigen_vector_sym_t q = create_symbolic_vector("q", model_nq());
    eigen_vector_sym_t v = create_symbolic_vector("v", model_nv());
    eigen_vector_sym_t a = create_symbolic_vector("a", model_nv());

    eigen_vector_sym_t w = create_symbolic_vector("w", dimension());
    eigen_vector_sym_t xacc_d = create_symbolic_vector("xacc_d", dimension());

    // Compute frame state
    frame_state<sym_t> frame =
        get_frame_state(model, q, v, a, target_frame, reference_frame);

    eigen_vector_sym_t dxacc = frame.acc.angular() - xacc_d;

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

CentreOfMassTask::CentreOfMassTask(const model_sym_t &model,
                                   const std::string &reference_frame) {
    eigen_vector_sym_t q = create_symbolic_vector("q", model_nq());
    eigen_vector_sym_t v = create_symbolic_vector("v", model_nv());

    pinocchio::DataTpl<sym_t> data(model);

    // Compute frame state
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::updateFramePlacements(model, data);

    if (model.getFrameId(reference_frame) == model.frames.size()) {
        assert("No reference frame");
    }

    typedef pinocchio::SE3Tpl<sym_t> se3_t;
    typedef pinocchio::MotionTpl<sym_t> motion_t;

    se3_t oMb = data.oMf[model.getFrameId(reference_frame)];

    pinocchio::centerOfMass(model, data, q, v, false);

    // Get data for centre of mass
    eigen_vector_sym_t com = oMb.actInv(data.com[0]),
                       com_vel = oMb.actInv(data.vcom[0]);

    // Create expression evaluators
    sym_t q_s = eigen_to_casadi<sym_elem_t>::convert(q);
    sym_t v_s = eigen_to_casadi<sym_elem_t>::convert(v);

    sym_t com_s = eigen_to_casadi<sym_elem_t>::convert(com);
    sym_t com_vel_s = eigen_to_casadi<sym_elem_t>::convert(com_vel);

    xpos = std::make_unique<expression_evaluator_t>(com_s,
                                                    sym_vector_t({q_s, v_s}));
    xvel = std::make_unique<expression_evaluator_t>(com_vel_s,
                                                    sym_vector_t({q_s, v_s}));
}

bopt::quadratic_cost<CentreOfMassTask::value_type>::shared_ptr
CentreOfMassTask::to_task_cost(const model_sym_t &model) const {
    eigen_vector_sym_t q = create_symbolic_vector("q", model_nq());
    eigen_vector_sym_t v = create_symbolic_vector("v", model_nv());
    eigen_vector_sym_t a = create_symbolic_vector("a", model_nv());

    eigen_vector_sym_t w = create_symbolic_vector("w", dimension());
    eigen_vector_sym_t xacc_d = create_symbolic_vector("xacc_d", dimension());

    pinocchio::DataTpl<sym_t> data(model);

    pinocchio::centerOfMass(model, data, q, v, a, false);

    auto oMb = data.oMf[model.getFrameId(reference_frame)];

    pinocchio::centerOfMass(model, data, q, v, a, false);

    eigen_vector_sym_t dxacc = oMb.actInv(data.acom[0]) - xacc_d;

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

SE3Task::SE3Task(const model_sym_t &model, const std::string &target,
                 const std::string &reference_frame)
    : Task(6, model.nq, model.nv),
      target_frame(target),
      reference_frame(reference_frame) {
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

    // Flatten rotation matrix
    eigen_vector_sym_t rotation_vec(Eigen::Map<eigen_vector_sym_t>(
        frame.pos.rotation().data(),
        frame.pos.rotation().cols() * frame.pos.rotation().rows()));

    eigen_vector_sym_t x(3 + 9);
    x << frame.pos.translation(), rotation_vec;

    sym_t x_s = eigen_to_casadi<sym_elem_t>::convert(x);
    sym_t xvel_s = eigen_to_casadi<sym_elem_t>::convert(frame.vel.toVector());

    xpos =
        std::make_unique<expression_evaluator_t>(x_s, sym_vector_t({q_s, v_s}));
    xvel = std::make_unique<expression_evaluator_t>(xvel_s,
                                                    sym_vector_t({q_s, v_s}));
}

bopt::quadratic_cost<SE3Task::value_type>::shared_ptr SE3Task::to_task_cost(
    const model_sym_t &model) const {
    eigen_vector_sym_t q = create_symbolic_vector("q", model_nq());
    eigen_vector_sym_t v = create_symbolic_vector("v", model_nv());
    eigen_vector_sym_t a = create_symbolic_vector("a", model_nv());

    eigen_vector_sym_t w = create_symbolic_vector("w", dimension());
    eigen_vector_sym_t xacc_d = create_symbolic_vector("xacc_d", dimension());

    // Compute frame state
    frame_state<sym_t> frame =
        get_frame_state(model, q, v, a, target_frame, reference_frame);

    eigen_vector_sym_t dxacc = frame.acc.toVector() - xacc_d;

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

}  // namespace osc