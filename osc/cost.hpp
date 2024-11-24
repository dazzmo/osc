#pragma once

#include <bopt/program.hpp>

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>
#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/common.hpp"

namespace osc {

class OSC;

template <class T>
struct cost_traits {
    typedef typename T::value_type value_type;
    typedef typename T::index_type index_type;
    typedef typename T::integer_type integer_type;
};

template <typename VectorType>
struct cost_parameters {
    // Cost weighting
    VectorType w;
};

/**
 * @brief
 *
 */
class Cost {
    friend class OSC;

   public:
    // Typedefs
    typedef double value_type;
    typedef std::size_t index_type;
    typedef int integer_type;

    Cost() {
        // Create parameters
    }

   protected:
    virtual void add_to_program(const model_sym_t &model,
                                OSC &osc_program) = 0;

   private:
};

class EffortSquaredCost : public Cost {
   public:
    EffortSquaredCost() {}

    void add_to_program(const model_sym_t &model, OSC &osc_program) override;

   private:
};

}  // namespace osc