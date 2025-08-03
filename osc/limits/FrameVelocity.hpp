#pragma once

#include "osc/Limit.hpp"

namespace osc {

class FrameVelocityLimit : public MotionLimit {
   public:
    FrameVelocityLimit(const String &frame, const Eigen::Ref<Vector> &lb,
                       const Eigen::Ref<Vector> &ub)
        : MotionLimit(lb.size()) {
        assert(lb.size() == ub.size());
    }

    void computeLimits(const State &state, const Real &dt, Eigen::Ref<Vector> lbA,
                       Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbx,
                       Eigen::Ref<Vector> ubx) const override {
        const Vector bias;
        bias = state.getFrameAcceleration(state.getFrameIndex(frame_));
        // todo - you need the frame velocity
        lbA = (-state.model().velocityLimit - state.v() - dt * bias);
        ubA = (state.model().velocityLimit - state.v() - dt * bias);
    }

    void computeJacobian(const State &state, const Real &dt,
                         Eigen::Ref<Matrix> A) const override {
        const Matrix J;
        J = state.getFrameJacobian(state.getFrameIndex(frame_));
        // todo - you need the frame velocity
        A = dt * J;
    }

   protected:
   private:
    String frame_;
};

}  // namespace osc