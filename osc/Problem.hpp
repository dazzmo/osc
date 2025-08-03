/**
 * @file osc.hpp
 * @author your name (you@domain.com)
 * @brief Weighted Operational Space Control
 * @version 0.1
 * @date 2024-11-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "osc/Constraint.hpp"
#include "osc/Limit.hpp"
#include "osc/State.hpp"
#include "osc/Task.hpp"

namespace osc {

/**
 * @brief Default Operational Space Controller
 *
 */
class OSCProgram {
   private:
    using IndexVector = std::list<Index>;

    enum OSCVariables { QACC, CTRL, CONTACT, HOLONOMIC, OSC_NUM_VARIABLES };
    enum OSCConstriants { DYNAMICS, FRICTION, LIMITS, OSC_NUM_CONSTRAINTS };

    std::list<Size, OSC_NUM_VARIABLES> vidx = {};
    std::list<Size, OSC_NUM_VARIABLES> vsz = {};

    std::list<Size, OSC_NUM_CONSTRAINTS> cidx = {};
    std::list<Size, OSC_NUM_CONSTRAINTS> csz = {};

    enum class BindingAction { NONE, ADD, REMOVE };

    template <typename T>
    struct Binding {
        Binding(const T &data, const BindingAction &action,
                const Real &t_action)
            : data(data), action(action), t_action(t_action) {}

        /// @brief Data of the binding
        T data;
        /// @brief Action to perform when t_action occurs
        BindingAction action;
        /// @brief Time at which binding was created [s]
        Real t_initial;
        /// @brief Time at which the action should occur [s]
        Real t_action;
    };

    struct ContactBinding : public Binding<ContactAbstract::SharedPtr> {
        ContactBinding(const ContactAbstract::SharedPtr &data,
                       const BindingAction &action, const Real &t_action,
                       const IndexVector &contact_indices)
            : Binding<ContactAbstract::SharedPtr>(data, action, t_action),
              contact_indices(contact_indices) {}

        /// @brief Indices of variables to use for the binding
        IndexVector contact_indices;
    };

    using MotionTaskBinding = Binding<MotionTask::SharedPtr>;
    using ActuationTaskBinding = Binding<ActuationTask::SharedPtr>;
    using MotionLimitBinding = Binding<MotionLimit::SharedPtr>;
    using ActuationLimitBinding = Binding<ActuationLimit::SharedPtr>;

    using ConstraintBinding = Binding<ConstraintAbstract::SharedPtr>;
    using ContactBinding = Binding<ContactAbstract::SharedPtr>;

    void computeProblemSize() {
        Size nx = 0;
        Size nc = 0;

        nx += state.nv() + state.nu();
        for (const auto &contact : contacts_) {
            nc += contact->getDimension();
            nx += contact->getDimension();
        }
    }

   public:
    OSCProgram(const State &state, const Size &nu) {
        // Create variables for acceleration
        addVariables("qacc", state.nv());
    }

    //   void init(const State &state, const String &solver = "qpoases", const
    //   QPSolver::Options &opts = {});

    // solve(const State &state);

    void addMotionTask(const MotionTask::SharedPtr &task, const Real &t,
                       const Real &duration = 0) {
        motion_tasks_.push_back(
            MotionTaskBinding(task, BindingAction::ADD, t + duration));
    }

    void addActuationTask(const ActuationTask::SharedPtr &task, const Real &t,
                          const Real &duration = 0) {
        // Determine indices of the variables used
        actuation_tasks_.push_back(
            ActuationTaskBinding(task, BindingAction::ADD, t + duration));
    }

    void addContact(const ContactAbstract::SharedPtr &contact, const Real &t,
                    const Real &duration = 0) {
        // Add new variables to the program

        // Determine indices of the variables used
        const auto indices = in_indices_[OSC_IN::OSC_IN_CONTACT];

        contacts_.push_back(ContactBinding(contact, BindingAction::ADD,
                                           t + duration, cindices));
    }

    void addMotionLimit(const MotionLimit::SharedPtr &limit, const Real &t,
                        const Real &duration = 0) {
        motion_limits_.push_back(
            MotionLimitBinding(limit, BindingAction::ADD, t + duration));
    }

    void addActuationLimit(const ActuationLimit::SharedPtr &limit,
                           const Real &t, const Real &duration = 0) {
        actuation_limits_.push_back(
            ActuationLimitBinding(limit, BindingAction::ADD, t + duration));
    }

    void addHolonomicConstraint(const ConstraintAbstract::SharedPtr &constraint,
                                const Real &t, const Real &duration = 0);

    template <typename Binding>
    void scheduleBindings(const Real &t, std::vector<Binding> &bindings) {
        for (auto it = bindings.begin(); it != bindings.end();) {
            // Loop through task bindings
            if (it->action == BindingAction::NONE) {
                // Evaluate the task as normal
            }

            if (it->action == BindingAction::ADD && t >= it->t_action) {
                // Add task at full strength
                it->action == BindingAction::NONE;
            } else {
                // Introduce the task/constraint at a linear rate
                const Real &t0 = it->t_initial;
                const Real &ta = it->t_action;
                const Real tau = (t - t0) / (ta - t0);
                it->data->setGain(tau);
            }

            if (it->action == BindingAction::REMOVE && t >= it->t_action) {
                // Remove the it
                it = tasks_.erase(it);
                continue;
            } else {
                // Reduce the task/constraint at a linear rate
                const Real &t0 = task->t_initial;
                const Real &ta = task->t_action;
                const Real tau = (t - t0) / (ta - t0);
                task->data->setGain(1.0 - tau);
            }
            ++it;
        }
    }

    void schedule(const Real &t) {
        // Compute the dimension of the problem
        scheduleBindings(t, tasks_);
        scheduleBindings(t, limits_);
        scheduleBindings(t, constraints_);
        scheduleBindings(t, contacts_);

        // Check if problem has changed size
    }

    void solve(const Real &t, const State &state, const Real &dt) {
        // Manage tasks and constraints
        schedule(t);

        Matrix A;
        Vector lbA, ubA;

        // Dynamics constraint
        A.block(ConstraintIndex::DYNAMICS, Index::QACC,
                ConstraintSize::DYNAMICS, VariableSize::QACC) =
            state.getInertiaMatrix();

        lbA.middleRows(ConstraintIndex::DYNAMICS, ConstraintSize::DYNAMICS) =
            -state.getCoriolisAndGravitationalBias();
        ubA.middleRows(ConstraintIndex::DYNAMICS, ConstraintSize::DYNAMICS) =
            -state.getCoriolisAndGravitationalBias();

        for (const auto &actuation : actuations_) {
            const Matrix Bi = actuation.data->computeJacobian(state);
            A.topRows(state.nv()).middleCols(state.nv(), actuation->indices) -=
                Bi;
        }

        // Add contact jacobians
        Size idx = 0;
        for (const auto &contact : contacts_) {
            const auto nc = contact->getFrictionCone()->numParameters();

            // Add contact dynamics
            const auto &ci = contact->contact_indices;

            A.block(Index::QACC, Index::QACC, state.nv(), state.nv()) -=
                contact->computeContactJacobian().transpose();

            idx += contact->getFrictionCone()->numParameters();
        }

        // For all tasks and constraints, add to program
        for (const auto &task : motion_tasks_) {
            auto Hi = H.topLeftCorner(0, 0, state.nv(), state.nv());
            task->addToQPObjective(state, H, g);
        }
        for (const auto &contact : contacts_) {
            // constraint index
            // constraint->addQPConstraint(state, H, g);
        }
        for (const auto &limit : limits_) {
            limit->addToQPConstraints(state, H, g);
        }

        // Solve with selected QP solver
    }

    // For a given variable set, return the values
    void getValue(const VariableVector &variable, Vector &values);

   private:
    std::vector<MotionTaskBinding> motion_tasks_;
    std::vector<ActuationTaskBinding> actuation_tasks_;

    std::vector<MotionLimitBinding> motion_limits_;
    std::vector<ActuationLimitBinding> actuation_limits_;

    std::vector<ContactBinding> contacts_;
};

}  // namespace osc
