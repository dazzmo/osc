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

  const FrictionConeModelAbstract::SharedPtr &frictionCone() const {
    return friction_cone_;
  }

  const String &frame() const { return frame_; }

  /**
   * @brief The transform from the frame to the contact frame
   *
   * @return const SE3&
   */
  const SE3 &contactTransform() const { return fMc_; }

  // todo - virtual casadi::Function toCasadiFunction() const = 0;

  Matrix6 graspMatrix(const SE3 &offset) const {
    Matrix6 G = Matrix6::Zero();
    G.block<3, 3>(0, 0) = offset.rotation();
    G.block<3, 3>(3, 3) = offset.rotation();
    G.block<3, 3>(3, 0) = skew(offset.translation()) * offset.rotation();
    return G;
  }

 protected:
  ContactAbstract() : frame_("") {}
  ContactAbstract(const String &frame, const Size &dimension,
                  const FrictionConeModelAbstract::SharedPtr &friction_cone,
                  const SE3 &fMc = SE3::Identity())
      : TaskAbstract(dimension),
        frame_(frame),
        fMc_(fMc),
        friction_cone_(friction_cone) {}

 private:
  String frame_{""};
  SE3 fMc_;

  Real mu_{1.0};
  Vector3 normal_{Vector3::UnitZ()};

  Matrix3 skew(const Vector3 &v) const {
    Matrix3 M = Matrix3::Zero();
    M(0, 1) =-v[2];
    M(0, 2) = v[1];
    M(1, 2) =-v[0];

    M(1, 0) = v[2];
    M(2, 0) =-v[1];
    M(2, 1) = v[0];
    
    return M;
  }

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
          const FrictionConeModelAbstract::SharedPtr &friction_cone,
          const SE3 &fMc = SE3::Identity())
      : ContactAbstract(frame, dimension, friction_cone, fMc) {}

 private:
  TargetType target_;
};

// todo - wrench contact

}  // namespace osc