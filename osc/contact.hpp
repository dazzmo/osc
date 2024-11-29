#pragma once

#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/holonomic.hpp"

namespace osc {

class OSC;

class AbstractLinearConstraint {
 public:
  virtual std::shared_ptr<bopt::linear_constraint<double>> to_constraint(
      const model_t &model) const = 0;
};

class AbstractContactConstraint : public HolonomicConstraint {
 public:
  AbstractContactConstraint(const model_t &model, const string_t &frame,
                            const index_t &dimension)
      : HolonomicConstraint(dimension), frame(frame) {
    // Ensure the frame exists on the model
    if (model.getFrameId(frame) == model.frames.size()) {
      assert("Model does not have specified frame");
    }
  }
  // Frame for the contact to occur
  string_t frame;

 private:
};

class AbstractFrictionContact : public AbstractContactConstraint {
 public:
  AbstractFrictionContact(const model_t &model, const string_t &frame,
                          const index_t &dimension)
      : AbstractContactConstraint(model, frame, dimension) {
    parameters.n = vector3_t::UnitZ();
    parameters.t = vector3_t::UnitX();
    parameters.b = vector3_t::UnitY();

    parameters.mu = 1.0;

    parameters.lambda_ub =
        vector_t::Constant(dimension, std::numeric_limits<double>::infinity());
    parameters.lambda_lb =
        vector_t::Constant(dimension, -std::numeric_limits<double>::infinity());
  }

  struct parameters {
    // Contact surface normal vector
    vector3_t n;
    // Contact surface tangent vector
    vector3_t t;
    // Contact surface binormal vector
    vector3_t b;

    double mu;

    vector_t lambda_ub;
    vector_t lambda_lb;
  };

  parameters parameters;
};

class FrictionContact3D : public AbstractFrictionContact {
 public:
  FrictionContact3D(const model_t &model, const string_t &frame)
      : AbstractFrictionContact(model, frame, 3) {}

  void jacobian(const model_t &model, data_t &data, matrix_t &J) override {
    jacobian_tpl<double>(model, data, J);
  }

  void bias_acceleration(const model_t &model, data_t &data,
                         vector_t &bias) override {
    bias_acceleration_tpl<double>(model, data, bias);
  }

  void jacobian(const model_sym_t &model, data_sym_t &data,
                matrix_sym_t &J) override {
    jacobian_tpl<sym_t>(model, data, J);
  }

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         vector_sym_t &bias) override {
    bias_acceleration_tpl<sym_t>(model, data, bias);
  }

 private:
  template <typename T>
  void jacobian_tpl(const pinocchio::ModelTpl<T> &model,
                    pinocchio::DataTpl<T> &data, Eigen::MatrixX<T> &J) {
    typename pinocchio::DataTpl<T>::Matrix6x Jfull =
        pinocchio::DataTpl<T>::Matrix6x::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, model.getFrameId(frame),
                                pinocchio::WORLD, Jfull);
    J = Jfull.topRows(3);
  }

  template <typename T>
  void bias_acceleration_tpl(const pinocchio::ModelTpl<T> &model,
                             pinocchio::DataTpl<T> &data,
                             Eigen::VectorX<T> &bias) {
    pinocchio::MotionTpl<T> acc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame), pinocchio::WORLD);

    bias = acc.linear();
  }
};

class FrictionConeConstraint : public AbstractLinearConstraint {
 public:
  FrictionConeConstraint(
      const std::shared_ptr<AbstractFrictionContact> &contact) {
    // Create parameters
    mu = bopt::variable("mu");
    n = bopt::create_variable_vector("n", 3);
    t = bopt::create_variable_vector("t", 3);
    b = bopt::create_variable_vector("b", 3);
  }

  // Friction coeffcient
  bopt::variable mu;
  // Normal vector for contact (in contact surface frame)
  std::vector<bopt::variable> n;
  // Tangent vector for contact (in contact surface frame)
  std::vector<bopt::variable> t;
  // Remaining vector for contact (in contact surface frame)
  std::vector<bopt::variable> b;

  // todo - could also make a specialised constraint (without codegen and
  // produce this)

  // Return linear constraint
  bopt::linear_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const override;
  const std::shared_ptr<AbstractFrictionContact> &contact() const {
    return contact_;
  }

 private:
  std::shared_ptr<AbstractFrictionContact> contact_;
};

class NoSlipConstraint : public AbstractLinearConstraint {
 public:
  NoSlipConstraint(const std::shared_ptr<AbstractContactConstraint> &contact)
      : contact_(contact) {
    epsilon = bopt::create_variable_vector("eps", contact->dimension);
  }

  // Slack variable for no-slip condition in linear constraint defining
  // contact
  std::vector<bopt::variable> epsilon;

  // Return linear constraint
  bopt::linear_constraint<double>::shared_ptr to_constraint(
      const model_t &model) const override;

 private:
  std::shared_ptr<AbstractContactConstraint> contact_;
};

// class NoSlipCost : public AbstractQuadraticCost {
//   NoSlipCost(const std::shared_ptr<AbstractContactConstraint> &contact) {}

//   // Slack variable for no-slip condition in linear constraint defining
//   // contact
//   std::vector<bopt::variable> xacc
// };

}  // namespace osc