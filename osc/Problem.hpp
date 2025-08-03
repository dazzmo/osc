#pragma once

#include "osc/Constraint.hpp"
#include "osc/Limit.hpp"
#include "osc/QPData.hpp"
#include "osc/QPSolver.hpp"
#include "osc/State.hpp"
#include "osc/Contact.hpp"
#include "osc/Actuation.hpp"
#include "osc/Task.hpp"

namespace osc {

/**
 * @brief Default Operational Space Controller
 *
 */
class OSCProgram {
   private:
    enum OSCVariables {
        /// @brief Generalised accelerations
        VAR_QACC = 0,
        /// @brief Control inputs
        VAR_CTRL,
        /// @brief Contact force variables
        VAR_CONTACT,
        /// @brief Holonomic virtual forces
        VAR_HOLONOMIC,
        NUM_VARIABLES
    };

    enum OSCConstraints {
        CON_DYNAMICS = 0,
        CON_FRICTION,
        CON_LIMITS,
        CON_HOLONOMIC,
        NUM_CONSTRAINTS
    };

    /// @brief Starting index of each variable
    std::array<Index, NUM_VARIABLES> vidx = {};
    /// @brief Sizes of each variable
    std::array<Size, NUM_VARIABLES> vsz = {};

    /// @brief Starting index of each constraint
    std::array<Index, NUM_CONSTRAINTS> cidx = {};
    /// @brief Sizes of each constraint
    std::array<Size, NUM_CONSTRAINTS> csz = {};

    Size num_variables_ = 0;
    Size num_constraints_ = 0;

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

    using TaskBinding = Binding<TaskAbstract::SharedPtr>;
    using LimitBinding = Binding<LimitAbstract::SharedPtr>;

    using ConstraintBinding = Binding<ConstraintAbstract::SharedPtr>;
    using ContactBinding = Binding<ContactAbstract::SharedPtr>;

    /**
     * @brief Computes the size of the program based on the given state, tasks
     * and constraints. Returns true if the program has changed size since the
     * last time this function was called.
     *
     * @param state
     * @return true
     * @return false
     */
    bool computeProblemSize(const State &state);

   public:
    OSCProgram(const State &state, const Size &nu);

    void init(const State &state, const String &solver = "qpoases",
              const QPSolver::Options &opts = {});

    /**
     * @brief Add a motion task to the problem that can be represented in the
     * form
     * \ddot{x} = J \ddot{q} + \dot{J} \dot{q}
     *
     * @param task
     * @param t
     * @param duration
     */
    void addMotionTask(const TaskAbstract::SharedPtr &task, const Real &t,
                       const Real &duration = 0);

    /**
     * @brief Add an actuation task to the problem that can be represented in
     * the form
     * \ddot{x} = J \ddot{u} + \dot{J} \dot{u}
     *
     * @param task
     * @param t
     * @param duration
     */
    void addActuationTask(const TaskAbstract::SharedPtr &task, const Real &t,
                          const Real &duration = 0);

    void addContact(const ContactAbstract::SharedPtr &contact, const Real &t,
                    const Real &duration = 0);

    void addMotionLimit(const LimitAbstract::SharedPtr &limit, const Real &t,
                        const Real &duration = 0);

    void addActuationLimit(const LimitAbstract::SharedPtr &limit, const Real &t,
                           const Real &duration = 0);

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
                it->action = BindingAction::NONE;
            } else {
                // Introduce the task/constraint at a linear rate
                const Real &t0 = it->t_initial;
                const Real &ta = it->t_action;
                const Real tau = (t - t0) / (ta - t0);
                it->data->setGain(tau);
            }

            if (it->action == BindingAction::REMOVE && t >= it->t_action) {
                // Remove the it
                it = bindings.erase(it);
                continue;
            } else {
                // Reduce the task/constraint at a linear rate
                const Real &t0 = it->t_initial;
                const Real &ta = it->t_action;
                const Real tau = (t - t0) / (ta - t0);
                it->data->setGain(1.0 - tau);
            }
            ++it;
        }
    }

    void schedule(const Real &t);
    void solve(const Real &t, const State &state, const Real &dt);

   private:
    std::vector<TaskBinding> motion_tasks_;
    std::vector<TaskBinding> actuation_tasks_;

    std::vector<LimitBinding> motion_limits_;
    std::vector<LimitBinding> actuation_limits_;

    std::vector<ContactBinding> contacts_;

    std::vector<ConstraintBinding> constraints_;

    std::vector<ActuationAbstract::SharedPtr> actuations_;

    std::unique_ptr<ConicData> conic_data_;
    std::unique_ptr<QPSolver> qp_solver_;
};

}  // namespace osc
