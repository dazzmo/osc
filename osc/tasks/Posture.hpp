
#pragma once

#include "osc/Task.hpp"

namespace osc {

/**
 * @brief Task for tracking a specified frame
 *
 */
class PostureTask : public MotionTask<Vector> {
   public:
    void computeError(const State &state, Eigen::Ref<Vector> e) override;

    void computeJacobian(const State &state, Eigen::Ref<Matrix> jac) override;

    void computeAccelerationBias(const State &state,
                                 Eigen::Ref<Vector> bias) override;

   private:
};

}  // namespace osc