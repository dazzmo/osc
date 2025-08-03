#pragma once

#include "osc/Task.hpp"

namespace osc {

struct CentreOfMassTarget {
    CentreOfMassTarget()
        : position(Vector3::Zero()),
          velocity(Vector3::Zero()),
          acceleration(Vector3::Zero()) {}
          
    CentreOfMassTarget(const Vector3 &position,
                       const Vector3 &velocity = Vector3::Zero(),
                       const Vector3 &acceleration = Vector3::Zero())
        : position(position), velocity(velocity), acceleration(acceleration) {}

    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
};

/**
 * @brief Task for tracking a specified frame
 *
 */
class CentreOfMassTask : public Task<CentreOfMassTarget> {
   public:
    void computeError(const State &state, Eigen::Ref<Vector> e,
                      Eigen::Ref<Vector> dot_e) const override;

    void computeJacobian(const State &state,
                         Eigen::Ref<Matrix> jac) const override;

    void computeAccelerationBias(const State &state,
                                 Eigen::Ref<Vector> bias) const override;

   private:
};

}  // namespace osc