#pragma once

#include "osc/tasks/motion.hpp"

namespace osc {

class CentreOfMassTask : public MotionTask<Vector3> {
   public:
    CentreOfMassTask(const model_t &model);

   private:
};

}  // namespace osc