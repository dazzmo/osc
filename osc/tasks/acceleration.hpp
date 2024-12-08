#pragma once

#include "osc/tasks/base.hpp"

namespace osc {

class AccelerationBounds : public BoundConstraint {
 public:
  AccelerationBounds(const index_t &na) : BoundConstraint(na) {}

 private:
};

}