#pragma once

#include "osc/Limit.hpp"

namespace osc {

class VelocityLimit : public MotionLimit {
   public:
    VelocityLimit(const Eigen::Ref<Vector> &lb, const Eigen::Ref<Vector> &ub)
        : MotionLimit(lb.size()) {
        assert(lb.size() == ub.size());
    }

    void computeLimits(const State &state, const Real &dt,
                       Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubA,
                       Eigen::Ref<Vector> lbx,
                       Eigen::Ref<Vector> ubx) const override {
        const Real sigma = std::min(1.0 / gain(), 1e9);
        lbA = sigma * (-state.model().velocityLimit - state.v());
        ubA = sigma * (state.model().velocityLimit - state.v());
    }

    void computeJacobian(const State &state, const Real &dt,
                         Eigen::Ref<Matrix> A) const override {
        A.setIdentity();
        A.diagonal().array() *= dt;
    }

   protected:
   private:
    Size dimension_;
};

}  // namespace osc