#include "osc/Problem.hpp"

namespace osc {

bool OSCProgram::computeProblemSize(const State &state) {
    Size nx_previous = num_variables_;
    Size nc_previous = num_constraints_;

    Size nx = 0;
    Size nc = 0;

    vsz[VAR_QACC] = state.nv();
    vsz[VAR_CTRL] = state.nv();

    csz[CON_DYNAMICS] = state.nv();
    for (const auto &contact : contacts_) {
        // Add friction constraints
        vsz[VAR_CONTACT] += contact.data->frictionCone()->numParameters();
        csz[CON_FRICTION] += contact.data->frictionCone()->numLPConstraints();
    }

    for (const auto &limit : motion_limits_) {
        csz[CON_LIMITS] += limit.data->numLPConstraints();
    }

    for (const auto &limit : actuation_limits_) {
        csz[CON_LIMITS] += limit.data->numLPConstraints();
    }

    for (const auto &constraint : constraints_) {
        csz[CON_HOLONOMIC] += constraint.data->numConstraints();
        vsz[VAR_HOLONOMIC] += constraint.data->numConstraints();
    }

    // Update indices
    vidx[VAR_QACC] = 0;
    vidx[VAR_CTRL] = vidx[VAR_QACC] + vsz[VAR_QACC];
    vidx[VAR_CONTACT] = vidx[VAR_CTRL] + vsz[VAR_CTRL];
    vidx[VAR_HOLONOMIC] = vidx[VAR_CONTACT] + vsz[VAR_CONTACT];

    cidx[CON_DYNAMICS] = 0;
    cidx[CON_FRICTION] = cidx[CON_DYNAMICS] + csz[CON_DYNAMICS];
    cidx[CON_LIMITS] = cidx[CON_FRICTION] + csz[CON_FRICTION];
    cidx[VAR_HOLONOMIC] = cidx[VAR_HOLONOMIC] + csz[CON_LIMITS];

    return (nx != nx_previous) && (nc != nc_previous);
}

OSCProgram::OSCProgram(const State &state, const Size &nu) {}

void OSCProgram::init(const State &state, const String &solver,
                      const QPSolver::Options &opts) {}

void OSCProgram::addMotionTask(const TaskAbstract::SharedPtr &task,
                               const Real &t, const Real &duration) {
    motion_tasks_.push_back(
        TaskBinding(task, BindingAction::ADD, t + duration));
}

void OSCProgram::addActuationTask(const TaskAbstract::SharedPtr &task,
                                  const Real &t, const Real &duration) {
    // Determine indices of the variables used
    actuation_tasks_.push_back(
        TaskBinding(task, BindingAction::ADD, t + duration));
}

void OSCProgram::addContact(const ContactAbstract::SharedPtr &contact,
                            const Real &t, const Real &duration) {
    // Add new variables to the program
    contacts_.push_back(
        ContactBinding(contact, BindingAction::ADD, t + duration));
}

void OSCProgram::addMotionLimit(const LimitAbstract::SharedPtr &limit,
                                const Real &t, const Real &duration) {
    motion_limits_.push_back(
        LimitBinding(limit, BindingAction::ADD, t + duration));
}

void OSCProgram::addActuationLimit(const LimitAbstract::SharedPtr &limit,
                                   const Real &t, const Real &duration) {
    actuation_limits_.push_back(
        LimitBinding(limit, BindingAction::ADD, t + duration));
}

void OSCProgram::addHolonomicConstraint(
    const ConstraintAbstract::SharedPtr &constraint, const Real &t,
    const Real &duration) {
    constraints_.push_back(
        ConstraintBinding(constraint, BindingAction::ADD, t + duration));
}

void OSCProgram::schedule(const Real &t) {
    // Compute the dimension of the problem
    scheduleBindings(t, motion_tasks_);
    scheduleBindings(t, actuation_tasks_);
    scheduleBindings(t, motion_limits_);
    scheduleBindings(t, actuation_limits_);
    scheduleBindings(t, constraints_);
    scheduleBindings(t, contacts_);
}

void OSCProgram::solve(const Real &t, const State &state, const Real &dt) {
    // Manage tasks and constraints
    schedule(t);
    if (computeProblemSize(state)) {
        // Create new problem with appropriate size
        conic_data_.reset();
        conic_data_ =
            std::make_unique<ConicData>(num_variables_, num_constraints_);
    }

    // Tracking index for variables in for loops
    Index idx_v = 0;
    // Tracking index for constraints in for loops
    Index idx_c = 0;

    // For all tasks and constraints, add to program
    for (const auto &task : motion_tasks_) {
        auto Hi = conic_data_->H.block(vidx[VAR_QACC], vidx[VAR_QACC],
                                       vsz[VAR_QACC], vsz[VAR_QACC]);
        auto gi = conic_data_->g.segment(vidx[VAR_QACC], vsz[VAR_QACC]);

        task.data->toQPObjective(state, Hi, gi);
    }

    for (const auto &task : actuation_tasks_) {
        auto Hi = conic_data_->H.block(vidx[VAR_CTRL], vidx[VAR_CTRL],
                                       vsz[VAR_CTRL], vsz[VAR_CTRL]);
        auto gi = conic_data_->g.segment(vidx[VAR_CTRL], vsz[VAR_CTRL]);

        task.data->toQPObjective(state, Hi, gi);
    }

    // Limits
    idx_c = 0;
    for (const auto &limit : motion_limits_) {
        const Size m = limit.data->numLPConstraints();
        auto lbx = conic_data_->lbx.middleRows(vidx[VAR_QACC], vsz[VAR_QACC]);
        auto ubx = conic_data_->ubx.middleRows(vidx[VAR_QACC], vsz[VAR_QACC]);

        auto A = conic_data_->A.block(cidx[CON_LIMITS] + idx_c, vidx[VAR_QACC],
                                      m, vsz[VAR_QACC]);
        auto lbA = conic_data_->lbA.segment(cidx[CON_LIMITS] + idx_c, m);
        auto ubA = conic_data_->ubA.segment(cidx[CON_LIMITS] + idx_c, m);

        limit.data->toLPConstraints(state, dt, A, lbA, ubA, lbx, ubx);
        idx_c += m;
    }
    idx_c = 0;
    for (const auto &limit : actuation_limits_) {
        const Size m = limit.data->numLPConstraints();
        auto lbx = conic_data_->lbx.middleRows(vidx[VAR_CTRL], vsz[VAR_CTRL]);
        auto ubx = conic_data_->ubx.middleRows(vidx[VAR_CTRL], vsz[VAR_CTRL]);

        auto A = conic_data_->A.block(cidx[CON_LIMITS] + idx_c, vidx[VAR_CTRL],
                                      m, vsz[VAR_CTRL]);
        auto lbA = conic_data_->lbA.segment(cidx[CON_LIMITS] + idx_c, m);
        auto ubA = conic_data_->ubA.segment(cidx[CON_LIMITS] + idx_c, m);

        limit.data->toLPConstraints(state, dt, A, lbA, ubA, lbx, ubx);
        idx_c += m;
    }

    // Dynamics constraint
    conic_data_->A.block(cidx[CON_DYNAMICS], vidx[VAR_QACC], csz[CON_DYNAMICS],
                         vsz[VAR_QACC]) = state.getInertiaMatrix();

    const Vector h = state.computeNonlinearEffects(state.q(), state.v());
    conic_data_->lbA.middleRows(cidx[CON_DYNAMICS], csz[CON_DYNAMICS]) = -h;
    conic_data_->ubA.middleRows(cidx[CON_DYNAMICS], csz[CON_DYNAMICS]) = -h;

    for (const auto &a : actuations_) {
        Matrix Bi = Matrix::Zero(a->numOutputs(), a->numInputs());
        a->computeJacobian(state, Bi);
        conic_data_->A.block(cidx[CON_DYNAMICS], vidx[VAR_CTRL],
                             csz[CON_DYNAMICS], vsz[VAR_CTRL]) -= Bi;
    }

    // Barriers

    // Constraints
    idx_v = 0;
    idx_c = 0;
    for (const auto &constraint : constraints_) {
        const Size n = constraint.data->numConstraints();
        const Size m = constraint.data->numConstraints();

        // Dynamics
        auto A = conic_data_->A.block(cidx[CON_DYNAMICS],
                                      vidx[VAR_HOLONOMIC] + idx_c,
                                      csz[CON_DYNAMICS], n);
        Matrix J;
        constraint.data->computeJacobian(state, J);
        A -= J.transpose();

        idx_v += n;
        idx_c += m;
    }

    // Contacts
    idx_v = 0;
    idx_c = 0;
    for (const auto &contact : contacts_) {
        const Size n = contact.data->frictionCone()->numParameters();
        const Size m = contact.data->frictionCone()->numLPConstraints();

        // Dynamics
        auto A =
            conic_data_->A.block(cidx[CON_DYNAMICS], vidx[VAR_CONTACT] + idx_c,
                                 csz[CON_DYNAMICS], n);
        Matrix J;
        contact.data->computeContactJacobian(state, J);
        J = contact.data->frictionCone()
                ->parameterisationToForceMap()
                .transpose() *
            J;
        A -= J.transpose();

        // Friction cones
        A = conic_data_->A.block(cidx[CON_FRICTION] + idx_c, vidx[VAR_QACC], m,
                                 vsz[VAR_QACC]);
        auto lbA = conic_data_->lbA.segment(cidx[CON_FRICTION] + idx_c, m);
        auto ubA = conic_data_->ubA.segment(cidx[CON_FRICTION] + idx_c, m);
        auto lbx = conic_data_->lbx.segment(vidx[VAR_CONTACT] + idx_v, n);
        auto ubx = conic_data_->ubx.segment(vidx[VAR_CONTACT] + idx_v, n);
        contact.data->frictionCone()->toLPConstraints(A, lbA, ubA, lbx, ubx);

        // Objective
        auto Hi = conic_data_->H.block(vidx[VAR_QACC], vidx[VAR_QACC],
                                       vsz[VAR_QACC], vsz[VAR_QACC]);
        auto gi = conic_data_->g.segment(vidx[VAR_QACC], vsz[VAR_QACC]);
        // contact.data->toQPObjective(state, Hi, gi);

        idx_c += m;
        idx_v += n;
    }

    // Solve with selected QP solver
    qp_solver_->solve(conic_data_->H, conic_data_->g, conic_data_->A,
                      conic_data_->ubA, conic_data_->lbA, conic_data_->ubx,
                      conic_data_->lbx);
}

}  // namespace osc
