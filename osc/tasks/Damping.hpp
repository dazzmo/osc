
#pragma once

#include "osc/Task.hpp"

namespace osc {

/**
 * @brief Task for damping velocities
 *
 */
class DampingTask : public Task<Vector> {
   public:
    DampingTask(const State &state);

    void computeError(const State &state, Eigen::Ref<Vector> e,
                      Eigen::Ref<Vector> dot_e) const override;

    void computeJacobian(const State &state,
                         Eigen::Ref<Matrix> jac) const override;

    void computeAccelerationBias(const State &state,
                                 Eigen::Ref<Vector> bias) const override;

   private:
};

}  // namespace osc