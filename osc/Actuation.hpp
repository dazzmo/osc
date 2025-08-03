#pragma once

namespace osc {

/**
 * @brief Transforms control inputs u into generalised forces.
 *
 */
class ActuationAbstract {
   public:
    void computeJacobian(const State &state, Eigen::Ref<Matrix> jac) = 0;

   private:
};

}  // namespace osc