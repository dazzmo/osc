
#pragma once

#include "osc/Limit.hpp"

namespace osc {

class AccelerationLimit : public LimitAbstract {
 public:
  AccelerationLimit(const State &state, const Eigen::Ref<Vector> &lb,
                    const Eigen::Ref<Vector> &ub)
      : LimitAbstract(state.nv()), lb_(lb), ub_(ub) {
    assert(lb.size() == ub.size());
  }
  void computeLimits(const State &state, const Real &dt, Eigen::Ref<Vector> lbA,
                     Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbx,
                     Eigen::Ref<Vector> ubx) const override {
    lbx = lb_;
    ubx = ub_;
  }

  void computeJacobian(const State &state, const Real &dt,
                       Eigen::Ref<Matrix> A) const override {}

 protected:
 private:
  Size dimension_;
  Vector lb_;
  Vector ub_;
};

}  // namespace osc