#pragma once

#include "osc/Types.hpp"

namespace osc {

struct ConicData {
    ConicData(const Size &nx, const Size &nc)
        : H(Matrix::Zero(nx, nx)),
          g(Vector::Zero(nx)),
          A(Matrix::Zero(nc, nx)),
          lbA(Vector::Constant(nc,-1e9)),
          ubA(Vector::Constant(nc, 1e9)),
          lbx(Vector::Constant(nx,-1e9)),
          ubx(Vector::Constant(nx, 1e9)) {}

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
