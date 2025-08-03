
#pragma once

#include "osc/Limit.hpp"

namespace osc {

class AccelerationLimit : public MotionLimit {
   public:
    AccelerationLimit(const State &state, const Eigen::Ref<Vector> &lb,
                      const Eigen::Ref<Vector> &ub)
        : MotionLimit(state.nv()) {
        assert(lb.size() == ub.size());
    }
    void computeLimits(const State &state, const Real &dt,
                       Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubA,
                       Eigen::Ref<Vector> lbx,
                       Eigen::Ref<Vector> ubx) const override {}

    void computeJacobian(const State &state, const Real &dt,
                         Eigen::Ref<Matrix> A) const override {}

   protected:
   private:
    Size dimension_;
    Vector lb_;
    Vector ub_;
};

}  // namespace osc