#pragma once

#include "osc/FrictionConeModel.hpp"
#include "osc/Fwd.hpp"
#include "osc/State.hpp"
#include "osc/Task.hpp"

namespace osc {

class ContactAbstract : public TaskAbstract {
   public:
    using SharedPtr = std::shared_ptr<ContactAbstract>;

    /**
     * @brief Set the surface normal of the contact point (within the world
     * frame)
     *
     * @param normal
     */
    void setSurfaceNormal(const Eigen::Vector3<Real> &normal) {
        normal_ = normal;
    }
    const Vector3 &surfaceNormal() const { return normal_; }

    void setFrictionCoefficient(const Real &mu) { mu_ = mu; }
    const Real &frictionCoefficient() { return mu_; }

    /**
     * @brief Computes the contact jacobian that maps a world-frame force to the
     * generalised inputs
     *
     * @param state
     * @param jac
     */
    virtual void computeContactJacobian(const State &state,
                                        Eigen::Ref<Matrix> jacobian) const = 0;

    const FrictionConeModelAbstract::SharedPtr &frictionCone() const {
        return friction_cone_;
    }

    const String &frame() const { return frame_; }

    // todo - virtual casadi::Function toCasadiFunction() const = 0;

   protected:
    ContactAbstract() : frame_("") {}
    ContactAbstract(const String &frame, const Size &dimension,
                    const FrictionConeModelAbstract::SharedPtr &friction_cone)
        : TaskAbstract(dimension),
          frame_(frame),
          friction_cone_(friction_cone) {}

   private:
    String frame_{""};

    Real mu_{1.0};
    Vector3 normal_{Vector3::UnitZ()};

    FrictionConeModelAbstract::SharedPtr friction_cone_{nullptr};
};

template <typename _TargetType>
class Contact : public ContactAbstract {
   public:
    using TargetType = _TargetType;

    void setTarget(const TargetType &target) { target_ = target; }
    virtual void setTargetFromState(const State &state) = 0;

    const TargetType &getTarget() const { return target_; }

   protected:
    Contact() {}
    Contact(const String &frame, const Size &dimension,
            const FrictionConeModelAbstract::SharedPtr &friction_cone)
        : ContactAbstract(frame, dimension, friction_cone) {}

   private:
    TargetType target_;
};

// todo - wrench contact

}  // namespace osc