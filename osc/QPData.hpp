#pragma once

#include "osc/Types.hpp"

namespace osc {

struct ConicData {
    ConicData(const Size &nx, const Size &nc) {}

    Matrix H;
    Vector g;
    Matrix A;

    Vector lbA;
    Vector ubA;

    Vector lbx;
    Vector ubx;

    Matrix Q;
    Vector P;
};

}  // namespace osc
