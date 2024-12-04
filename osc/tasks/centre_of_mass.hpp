#pragma once

#include "osc/tasks/task.hpp"

namespace osc {

#define CENTRE_OF_MASS_TASK_DIM (3)

class CentreOfMassTask : public AbstractTask {
 public:
  CentreOfMassTask(const model_t &model,
                   const std::string &reference_frame = "universe")
      : AbstractTask(CENTRE_OF_MASS_TASK_DIM) {}

  struct reference {
    vector3_t position;
    vector3_t velocity;
  };

  void jacobian(const model_t &model, data_t &data, const vector_t &q,
                matrix_t &J) const override;

  void bias_acceleration(const model_t &model, data_t &data, const vector_t &q,
                         const vector_t &v, vector_t &bias) const override;

  void jacobian(const model_sym_t &model, data_sym_t &data,
                const vector_sym_t &q, matrix_sym_t &J) const override;

  void bias_acceleration(const model_sym_t &model, data_sym_t &data,
                         const vector_sym_t &q, const vector_sym_t &v,
                         vector_sym_t &bias) const override;

  reference target;
  string_t reference_frame;

  void evaluate_error(const model_t &model, data_t &data, vector_t &e,
                      vector_t &e_dot) const override;

 private:
  template <typename T>
  void evaluate_tpl(const pinocchio::ModelTpl<T> &model,
                    const pinocchio::DataTpl<T> &data,
                    const Eigen::VectorX<T> &q, Eigen::MatrixX<T> &J,
                    Eigen::VectorX<T> &dJdq) {
    pinocchio::jacobianCenterOfMass(model, data, J);
    dJdq = data.acom[0];
  }

 private:
  template <typename T>
  void jacobian_tpl(const pinocchio::ModelTpl<T> &model,
                    pinocchio::DataTpl<T> &data, const Eigen::VectorX<T> &q,
                    Eigen::MatrixX<T> &J) const {
    J = pinocchio::jacobianCenterOfMass(model, data);
  }

  template <typename T>
  void bias_acceleration_tpl(const pinocchio::ModelTpl<T> &model,
                             pinocchio::DataTpl<T> &data,
                             const Eigen::VectorX<T> &q,
                             const Eigen::VectorX<T> &v,
                             Eigen::VectorX<T> &bias) const {
    bias = data.acom[0];
  }
};

}  // namespace osc