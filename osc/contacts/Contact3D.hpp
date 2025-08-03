#pragma once
#include "osc/Contact.hpp"

namespace osc {

class Contact3D : public Contact<Vector3> {
    static constexpr int DIMENSION = 3;

   public:
    Contact3D(const String &frame,
                 FrictionConeModelAbstract::SharedPtr &friction_cone)
        : Contact<Vector3>(frame, DIMENSION, friction_cone) {}

    void computeError(const State &state, Eigen::Ref<Vector> e, Eigen::Ref<Vector> dot_e) const override;

    void computeJacobian(const State &state,
                         Eigen::Ref<Matrix> jacobian) const override;

    void computeAccelerationBias(const State &state,
                                 Eigen::Ref<Vector> bias) const override;

    void computeContactJacobian(const State &state,
                                Eigen::Ref<Matrix> jacobian) const override;

   private:
};

}  // namespace osc
