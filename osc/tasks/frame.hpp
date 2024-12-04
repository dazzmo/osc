
#pragma once

#include "osc/task/task.hpp"

namespace osc {

/**
 * @brief Task for tracking a specified frame
 * 
 */
class FrameTask : public AbstractTask {
 public:
  enum class Type { Position = 0, Orientation, Full };

  FrameTask(const model_t &model, const std::string &frame_name,
            const Type &type = Type::Full,
            const std::string &reference_frame = "universe");

  static std::shared_ptr<FrameTask> create(
      const model_t &model, const std::string &frame_name,
      const Type &type = Type::Full,
      const std::string &reference_frame = "universe");

  void jacobian(const model_t &model, data_t &data, const vector_t &q,
                matrix_t &J) const override;

  void bias_acceleration(const model_t &model, data_t &data, const vector_t &q,
                         const vector_t &v, vector_t &bias) const override;

  void jacobian(const model_sym_t &model, data_sym_t &data,
                const vector_sym_t &q, matrix_sym_t &J) const override;

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         const vector_sym_t &q, const vector_sym_t &v,
                         vector_sym_t &bias) const override;

  Type type;

  string_t frame_name;
  string_t reference_frame;

  struct reference {
    pinocchio::SE3 pose = pinocchio::SE3::Identity();
    pinocchio::Motion twist = pinocchio::Motion::Zero();
  };

  reference target;

  void evaluate_error(const model_t &model, data_t &data, vector_t &e,
                      vector_t &e_dot) const override;

 private:
  template <typename T>
  void jacobian_tpl(const pinocchio::ModelTpl<T> &model,
                    pinocchio::DataTpl<T> &data, const Eigen::VectorX<T> &q,
                    Eigen::MatrixX<T> &J) const {
    typename pinocchio::DataTpl<T>::Matrix6x Jfull =
        pinocchio::DataTpl<T>::Matrix6x::Zero(6, model.nv);

    pinocchio::getFrameJacobian(model, data, model.getFrameId(frame_name),
                                pinocchio::LOCAL, Jfull);
    if (type == Type::Position) {
      J = Jfull.topRows(3);
    } else if (type == Type::Orientation) {
      J = Jfull.bottomRows(3);
    } else if (type == Type::Full) {
      J = Jfull;
    }
  }

  template <typename T>
  void bias_acceleration_tpl(const pinocchio::ModelTpl<T> &model,
                             pinocchio::DataTpl<T> &data,
                             const Eigen::VectorX<T> &q,
                             const Eigen::VectorX<T> &v,
                             Eigen::VectorX<T> &bias) const {
    pinocchio::MotionTpl<T> acc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame_name));

    if (type == Type::Position) {
      bias = acc.linear();
    } else if (type == Type::Orientation) {
      bias = acc.angular();
    } else if (type == Type::Full) {
      bias = acc.toVector();
    }
  }
};

}  // namespace osc