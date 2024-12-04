#include "osc/tasks/centre_of_mass.hpp"

namespace osc {

void CentreOfMassTask::jacobian(const model_t &model, data_t &data,
                                const vector_t &q, matrix_t &J) const {
  jacobian_tpl<double>(model, data, q, J);
}

void CentreOfMassTask::bias_acceleration(const model_t &model, data_t &data,
                                         const vector_t &q, const vector_t &v,
                                         vector_t &bias) const {
  bias_acceleration_tpl<double>(model, data, q, v, bias);
}

void CentreOfMassTask::jacobian(const model_sym_t &model, data_sym_t &data,
                                const vector_sym_t &q, matrix_sym_t &J) const {
  jacobian_tpl<sym_t>(model, data, q, J);
}

void CentreOfMassTask::bias_acceleration(const model_sym_t &model,
                                         data_sym_t &data,
                                         const vector_sym_t &q,
                                         const vector_sym_t &v,
                                         vector_sym_t &bias) const {
  bias_acceleration_tpl<sym_t>(model, data, q, v, bias);
}

void CentreOfMassTask::evaluate_error(const model_t &model, data_t &data,
                                      vector_t &e, vector_t &e_dot) const {
  // Compute centre of mass with respect to reference frame
  e = data.oMf[model.getFrameId(reference_frame)].actInv(data.com[0]) -
      target.position;
  // Also compute centre of mass velocity
  e_dot = data.oMf[model.getFrameId(reference_frame)].actInv(data.vcom[0]) -
          target.velocity;
}

}  // namespace osc
