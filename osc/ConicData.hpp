#pragma once

#include "osc/Types.hpp"

namespace osc {

struct ConicData {
    ConicData(const Size &nx, const Size &nc)
        : H(Matrix::Zero(nx, nx)),
          g(Vector::Zero(nx)),
          A(Matrix::Zero(nc, nx)),
          lbA(Vector::Zero(nc)),
          ubA(Vector::Zero(nc)),
          lbx(Vector::Zero(nx)),
          ubx(Vector::Zero(nx)) {}

    Matrix H;
    Vector g;
    Matrix A;

    Vector lbA;
    Vector ubA;

    Vector lbx;
    Vector ubx;

    // todo
    Matrix Q;
    Vector P;
};

}  // namespace osc
