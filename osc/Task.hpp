#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

class TaskAbstract {
 public:
  using SharedPtr = std::shared_ptr<TaskAbstract>;
  /**
   * @brief Returns the dimension of the task.
   * @note This is the dimension of the task error, not necessarily the
   * dimension of the task. For the dimension of the task see
   * getTaskDimension()
   *
   * @return Size
   */
  Size getDimension() const { return dimension_; }
  Size getTaskDimension() const { return task_dimension_; }

  /**
   * @brief Set the effective gain of the task
   *
   * @param gain
   */
  void setGain(const Real &gain) { gain_ = gain; }
  const Real &gain() const { return gain_; }

  const Vector &weighting() const { return weighting_; }
  void setWeighting(const Eigen::Ref<Vector> &weighting) {
    weighting_ = weighting;
  }
  void setWeighting(const Real &weighting) {
    weighting_.setConstant(weighting);
  }

  /**
   * @brief Computes the task error and its rate.
   *
   * @param state
   * @param e
   */
  virtual void computeError(const State &state, Eigen::Ref<Vector> e,
                            Eigen::Ref<Vector> dot_e) const = 0;

  /**
   * @brief Computes the task Jacobian
   *
   * @param state
   * @param jac
   */
  virtual void computeJacobian(const State &state,
                               Eigen::Ref<Matrix> jac) const = 0;
  /**
   * @brief Computes the task acceleration bias term, given as \gamma =
   * \dot{J}(q) \dot{q}
   *
   * @param state
   * @param jac
   */
  virtual void computeAccelerationBias(const State &state,
                                       Eigen::Ref<Vector> bias) const = 0;

  void addToQPObjective(const State &state, Eigen::Ref<Matrix> H,
                        Eigen::Ref<Vector> g) const {
    Vector e(getDimension()), dot_e(getDimension()), bias(getDimension());
    computeError(state, e, dot_e);
    Matrix J(getDimension(), H.rows()), W(getDimension(), getDimension());
    computeJacobian(state, J);
    computeAccelerationBias(state, bias);
    W = weighting().asDiagonal();

    // Compute the desired task acceleration to minimise the error
    const Vector xacc = computeDesiredAcceleration(e, dot_e);
    std::cout << "xacc = " << xacc << std::endl;

    const Matrix A = W * J;
    const Vector b = W * (bias - xacc);

    // Compute weighting
    H += gain() * A.transpose() * A;
    g += gain() * A.transpose() * b;
  }

  void setErrorPD(const Vector &Kp, const Vector &Kd) {
    Kp_ = Kp;
    Kd_ = Kd;
  }

  Vector computeDesiredAcceleration(const Eigen::Ref<Vector> &e,
                                    const Eigen::Ref<Vector> &dot_e) const {
    // todo - plus desired xacc
    return -(Kp_.asDiagonal() * e + Kd_.asDiagonal() * dot_e);
  }

 protected:
  TaskAbstract()
      : dimension_(0),
        task_dimension_(0),
        gain_(1.0),
        weighting_(Vector::Ones(0)),
        Kp_(Vector::Ones(0)),
        Kd_(Vector::Ones(0)) {}

  TaskAbstract(const Size &dimension)
      : dimension_(dimension),
        task_dimension_(dimension),
        gain_(1.0),
        weighting_(Vector::Ones(dimension)),
        Kp_(Vector::Ones(dimension)),
        Kd_(Vector::Ones(dimension)) {}

  void setDimension(const Size &dimension) { dimension_ = dimension; }
  void setTaskDimension(const Size &dimension) { task_dimension_ = dimension; }

 private:
  /// @brief Dimension of the error of the task
  Size dimension_;
  Size task_dimension_;

  Real gain_;
  Vector weighting_;

  Vector Kp_;
  Vector Kd_;
};

template <typename _TargetType>
class Task : public TaskAbstract {
 public:
  using TargetType = _TargetType;

  Task(const Size &dimension) : TaskAbstract(dimension) {}

  void setTarget(const TargetType &target) { target_ = target; }
  const TargetType &getTarget() const { return target_; }

 private:
  TargetType target_;
};

}  // namespace osc