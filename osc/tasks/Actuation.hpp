#pragma once

#include "osc/tasks/base.hpp"

namespace osc {

/**
 * @brief Task designed to minimise the value of ||u||^2
 *
 */
class MinimiseActuationTask : public ActuationTask<Vector> {
   public:

    void compute(const model_t &model, data_t &data, const vector_t &q,
                 const vector_t &v) override {
        computeJacobian(model, data, q, v);
    }

    void computeJacobian(const model_t &model, data_t &data, const vector_t &q,
                          const vector_t &v) override {
        jacobian_.setIdentity();
    }

   protected:
   private:
    index_t nu_;
};

}  // namespace osc