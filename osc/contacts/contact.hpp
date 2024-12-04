#pragma once

#include <bopt/ad/casadi.hpp>
#include <bopt/constraints.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/constraint.hpp"
#include "osc/holonomic.hpp"

namespace osc {

class AbstractContact : public HolonomicConstraint {
 public:
  AbstractContact(const model_t &model, const string_t &frame,
                  const index_t &dimension)
      : HolonomicConstraint(dimension), frame(frame) {
    // Ensure the frame exists on the model
    if (model.getFrameId(frame) == model.frames.size()) {
      assert("Model does not have specified frame");
    }
  }
  // Frame for the contact to occur
  string_t frame;

  enum class ContactState { CONTACT = 0, NO_CONTACT };

  ContactState contact;

 private:
};

class AbstractFrictionContact : public AbstractContact {
 public:
  AbstractFrictionContact(const model_t &model, const string_t &frame,
                          const index_t &dimension)
      : AbstractContact(model, frame, dimension) {
    n = vector3_t::UnitZ();
    t = vector3_t::UnitX();
    b = vector3_t::UnitY();

    mu = 1.0;

    lambda_ub =
        vector_t::Constant(dimension, std::numeric_limits<double>::infinity());
    lambda_lb =
        vector_t::Constant(dimension, -std::numeric_limits<double>::infinity());
  }

  // Contact surface normal vector
  vector3_t n;
  // Contact surface tangent vector
  vector3_t t;
  // Contact surface binormal vector
  vector3_t b;

  // Friction coefficient
  double mu;

  // Friction force upper bound
  vector_t lambda_ub;
  // Friction force lower bound
  vector_t lambda_lb;
};

class FrictionContact3D : public AbstractFrictionContact {
 public:
  FrictionContact3D(const model_t &model, const string_t &frame)
      : AbstractFrictionContact(model, frame, 3) {}

  void jacobian(const model_t &model, data_t &data, const vector_t &q,
                matrix_t &J) const override {
    jacobian_tpl<double>(model, data, q, J);
  }

  void bias_acceleration(const model_t &model, data_t &data, const vector_t &q,
                         const vector_t &v, vector_t &bias) const override {
    bias_acceleration_tpl<double>(model, data, q, v, bias);
  }

  void jacobian(const model_sym_t &model, data_sym_t &data,
                const vector_sym_t &q, matrix_sym_t &J) const override {
    jacobian_tpl<sym_t>(model, data, q, J);
  }

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         const vector_sym_t &q, const vector_sym_t &v,
                         vector_sym_t &bias) const override {
    bias_acceleration_tpl<sym_t>(model, data, q, v, bias);
  }

 private:
  template <typename T>
  void jacobian_tpl(const pinocchio::ModelTpl<T> &model,
                    pinocchio::DataTpl<T> &data, const Eigen::VectorX<T> &q,
                    Eigen::MatrixX<T> &J) const {
    typename pinocchio::DataTpl<T>::Matrix6x Jfull =
        pinocchio::DataTpl<T>::Matrix6x::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, model.getFrameId(frame),
                                pinocchio::WORLD, Jfull);
    J = Jfull.topRows(3);
  }

  template <typename T>
  void bias_acceleration_tpl(const pinocchio::ModelTpl<T> &model,
                             pinocchio::DataTpl<T> &data,
                             const Eigen::VectorX<T> &q,
                             const Eigen::VectorX<T> &v,
                             Eigen::VectorX<T> &bias) const {
    pinocchio::MotionTpl<T> acc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame), pinocchio::WORLD);

    bias = acc.linear();
  }
};

}  // namespace osc