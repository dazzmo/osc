#pragma once

#include <pinocchio/algorithm/joint-configuration.hpp>

#include "osc/Limit.hpp"

namespace osc {

class ConfigurationLimit : public LimitAbstract {
   public:
    ConfigurationLimit(const State &state) : LimitAbstract(state.nv()) {}

    void computeLimits(const State &state, const Real &dt,
                       Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubA,
                       Eigen::Ref<Vector> lbx,
                       Eigen::Ref<Vector> ubx) const override {
        lbA = (pinocchio::difference(state.model(), state.q(),
                                     state.model().lowerPositionLimit) -
               dt * state.model().velocityLimit);

        ubA = (pinocchio::difference(state.model(), state.q(),
                                     state.model().upperPositionLimit) -
               dt * state.model().velocityLimit);
    }

    void computeJacobian(const State &state, const Real &dt,
                         Eigen::Ref<Matrix> A) const override {
        A.setIdentity();
        A.diagonal().array() *= (0.5 * dt * dt);
    }

   protected:
   private:
};

}  // namespace osc
