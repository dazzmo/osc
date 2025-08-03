#include "osc/Problem.hpp"

namespace osc {

void OSCProgram::addTask(const TaskAbstract::SharedPtr &task,
                         const VariableVector &variables,
                         const Real &duration) {}

void OSCProgram::addContact(const ContactAbstract::SharedPtr &task,
                            const VariableVector &variables,
                            const Real &duration) {}

void OSCProgram::addLimit(const LimitAbstract::SharedPtr &limit,
                          const VariableVector &variables,
                          const Real &duration) {}

void OSCProgram::addConstraint(const ConstraintAbstract::SharedPtr &constraint,
                               const VariableVector &variables,
                               const Real &duration) {}

void OSCProgram::removeTask(const TaskAbstract::SharedPtr &task,
                            const Real &duration) {}

void OSCProgram::removeContact(const ContactAbstract::SharedPtr &task,
                               const Real &duration) {}

void OSCProgram::removeLimit(const LimitAbstract::SharedPtr &limit,
                             const Real &duration) {}

void OSCProgram::removeConstraint(
    const ConstraintAbstract::SharedPtr &constraint, const Real &duration) {}

}  // namespace osc
