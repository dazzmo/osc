#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

class LimitAbstract {
   public:
    using SharedPtr = std::shared_ptr<LimitAbstract>;

    /**
     * @brief The gain of the limit. If set to zero, the limit is effectively
     * unbounded. If set to 1 or higher, enforces the limit at full strength.
     *
     * @param gain
     */
    void setGain(const Real &gain) { gain_ = gain; }
    const Real &gain() const { return gain_; }

    virtual void computeLimits(const State &state, const Real &dt,
                               Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubA,
                               Eigen::Ref<Vector> lbx,
                               Eigen::Ref<Vector> ubx) const = 0;

    virtual void computeJacobian(const State &state, const Real &dt,
                                 Eigen::Ref<Matrix> A) const = 0;

    virtual Size numLPConstraints() const { return m_; }

    virtual void toLPConstraints(const State &state, const Real &dt,
                                 Eigen::Ref<Matrix> A, Eigen::Ref<Vector> lbA,
                                 Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbx,
                                 Eigen::Ref<Vector> ubx) const {
        computeLimits(state, dt, lbA, ubA, lbx, ubx);
        computeJacobian(state, dt, A);

        // Add the gain to the limits. Here a gain of zero will cause the bounds
        // to expand to infinity.
        Real sigma = std::min(Real(1) / gain(), 1e9);
        lbA *= sigma;
        ubA *= sigma;
        lbx *= sigma;
        ubx *= sigma;
    }

    // todo - virtual casadi::Function toCasadiFunction() const = 0;

   protected:
    LimitAbstract() : m_(0), gain_(1.0) {}
    LimitAbstract(const Size &m) : m_(m), gain_(1.0) {}

    void setDimension(const Size &m) { m_ = m; }

   private:
    Size m_;
    Real gain_;
};

}  // namespace osc