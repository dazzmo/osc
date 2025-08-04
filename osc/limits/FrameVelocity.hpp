#pragma once

#include "osc/Limit.hpp"

namespace osc {

class FrameVelocityLimit : public LimitAbstract {
    static constexpr Size DIMENSION = 6;

   public:
    FrameVelocityLimit(const String &frame, const Motion &lb, const Motion &ub)
        : LimitAbstract(DIMENSION), frame_(frame), lb_(lb), ub_(ub) {}

    void computeLimits(const State &state, const Real &dt,
                       Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubA,
                       Eigen::Ref<Vector> lbx,
                       Eigen::Ref<Vector> ubx) const override {
        Motion bias =
            state.getFrameClassicalAcceleration(state.getFrameIndex(frame_));
        lbA = (lb_ - state.getFrameVelocity(state.getFrameIndex(frame_)) -
               dt * bias)
                  .toVector();
        ubA = (ub_ - state.getFrameVelocity(state.getFrameIndex(frame_)) -
               dt * bias)
                  .toVector();
    }

    void computeJacobian(const State &state, const Real &dt,
                         Eigen::Ref<Matrix> A) const override {
        const Matrix J =
            state.computeFrameJacobian(state.getFrameIndex(frame_));
        A = dt * J;
    }

   protected:
   private:
    String frame_;
    Motion lb_;
    Motion ub_;
};

}  // namespace osc