#include "osc/contact.hpp"
#include "osc/osc.hpp"

namespace osc {

bopt::linear_constraint<ContactPoint3D::value_type>::shared_ptr
ContactPoint3D::create_friction_constraint(const model_sym_t &model) const {
    // https://scaron.info/robotics/friction-cones.html

    // Create friction constraint
    eigen_vector_sym_t f = create_symbolic_vector("f", dimension());
    eigen_vector_sym_t n = create_symbolic_vector("n", dimension());
    eigen_vector_sym_t t = create_symbolic_vector("t", dimension());
    eigen_vector_sym_t b = create_symbolic_vector("b", dimension());
    sym_t mu = sym_t::sym("mu");

    // Inner approximation of friction cone
    sym_t mu_tilde = mu / sqrt(2.0);

    sym_t limit = mu * f.dot(n);

    // Constraint to indicate | f.dot(t) | <= mu * f.dot(n)
    sym_t constraint = sym_t::zeros(4);
    constraint(0) = f.dot(t) - limit;
    constraint(1) = -f.dot(t) - limit;
    constraint(2) = f.dot(b) - limit;
    constraint(3) = -f.dot(b) - limit;

    sym_t f_s = eigen_to_casadi<sym_elem_t>::convert(f);
    sym_t n_s = eigen_to_casadi<sym_elem_t>::convert(n);
    sym_t t_s = eigen_to_casadi<sym_elem_t>::convert(t);
    sym_t b_s = eigen_to_casadi<sym_elem_t>::convert(b);

    return bopt::casadi::linear_constraint<value_type>::create(
        constraint, f_s, sym_vector_t({n_s, t_s, b_s, mu}),
        bopt::bound_type::Negative);
}

bopt::bounding_box_constraint<ContactPoint3D::value_type>::shared_ptr
ContactPoint3D::create_friction_bound_constraint(
    const model_sym_t &model) const {
    // Create bounding box constraint
    eigen_vector_sym_t fl = create_symbolic_vector("fl", dimension());
    eigen_vector_sym_t fu = create_symbolic_vector("fu", dimension());

    sym_t fl_s = eigen_to_casadi<sym_elem_t>::convert(fl);
    sym_t fu_s = eigen_to_casadi<sym_elem_t>::convert(fu);

    return bopt::casadi::bounding_box_constraint<value_type>::create(
        fl_s, fu_s, sym_vector_t({fl_s, fu_s}));
}

bopt::linear_constraint<ContactPoint3D::value_type>::shared_ptr
ContactPoint3D::create_no_slip_constraint(const model_sym_t &model) const {
    // Compute the target frame in the contact frame of the model
    eigen_vector_sym_t q = create_symbolic_vector("q", model.nq);
    eigen_vector_sym_t v = create_symbolic_vector("v", model.nv);
    eigen_vector_sym_t a = create_symbolic_vector("a", model.nv);
    eigen_vector_sym_t e = create_symbolic_vector("e", dimension());

    // Compute frame state
    pinocchio::DataTpl<sym_t> data(model);

    // Compute the kinematic tree state of the system
    pinocchio::forwardKinematics(model, data, q, v, a);
    pinocchio::updateFramePlacements(model, data);

    // Compute the acceleration of the target frame in its LOCAL frame
    pinocchio::MotionTpl<sym_t> acc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame), pinocchio::WORLD);

    // Create expression evaluators
    sym_t q_s = eigen_to_casadi<sym_elem_t>::convert(q);
    sym_t v_s = eigen_to_casadi<sym_elem_t>::convert(v);
    sym_t a_s = eigen_to_casadi<sym_elem_t>::convert(a);
    sym_t e_s = eigen_to_casadi<sym_elem_t>::convert(e);

    // No slip condition => xacc = J qacc + \dot J qvel = epsilon
    sym_t constraint = eigen_to_casadi<sym_elem_t>::convert(acc.linear() - e);

    // todo - need to consider the frame transformation from contact frame to
    // todo - end-effector frame

    return bopt::casadi::linear_constraint<value_type>::create(
        constraint, a_s, sym_vector_t({q_s, v_s, e_s}));
}

void ContactPoint3D::add_to_program(const model_sym_t &model,
                                    OSC &osc_program) {
    // Parameters
    for (std::size_t i = 0; i < parameters_v.epsilon.size(); ++i) {
        osc_program.program.add_parameter(parameters_v.epsilon[i]);
    }
    for (std::size_t i = 0; i < parameters_v.n.size(); ++i) {
        osc_program.program.add_parameter(parameters_v.n[i]);
    }
    for (std::size_t i = 0; i < parameters_v.t.size(); ++i) {
        osc_program.program.add_parameter(parameters_v.t[i]);
    }
    for (std::size_t i = 0; i < parameters_v.b.size(); ++i) {
        osc_program.program.add_parameter(parameters_v.b[i]);
    }
    for (std::size_t i = 0; i < parameters_v.lambda_ub.size(); ++i) {
        osc_program.program.add_parameter(parameters_v.lambda_ub[i]);
    }
    for (std::size_t i = 0; i < parameters_v.lambda_lb.size(); ++i) {
        osc_program.program.add_parameter(parameters_v.lambda_lb[i]);
    }
    osc_program.program.add_parameter(parameters_v.mu);

    // Create constraints
    auto friction_cone = create_friction_constraint(model);
    auto friction_bound = create_friction_bound_constraint(model);
    auto no_slip = create_no_slip_constraint(model);

    // Bind to program
    osc_program.program.add_linear_constraint(
        friction_cone,
        // Variables
        {eigen_to_std_vector<bopt::variable>::convert(
            osc_program.variables.lambda.bottomRows(dimension()))},
        // contact-specific parameters
        {eigen_to_std_vector<bopt::variable>::convert(parameters_v.n),
         eigen_to_std_vector<bopt::variable>::convert(parameters_v.t),
         eigen_to_std_vector<bopt::variable>::convert(parameters_v.b),
         {parameters_v.mu}});

    osc_program.program.add_linear_constraint(
        no_slip,
        // Variables
        {eigen_to_std_vector<bopt::variable>::convert(osc_program.variables.a)},
        // Parameters
        {eigen_to_std_vector<bopt::variable>::convert(osc_program.parameters.q),
         eigen_to_std_vector<bopt::variable>::convert(osc_program.parameters.v),
         // Task-specific parameters
         eigen_to_std_vector<bopt::variable>::convert(parameters_v.epsilon)});

    osc_program.program.add_bounding_box_constraint(
        friction_bound,
        // Variables
        {eigen_to_std_vector<bopt::variable>::convert(
            osc_program.variables.lambda.bottomRows(dimension()))},
        // Parameters
        {eigen_to_std_vector<bopt::variable>::convert(parameters_v.lambda_lb),
         eigen_to_std_vector<bopt::variable>::convert(parameters_v.lambda_ub)});
}

}  // namespace osc