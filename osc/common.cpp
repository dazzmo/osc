#include "osc/common.hpp"

namespace osc {

vector_sym_t create_symbolic_vector(const string_t &name, const index_t &sz) {
  vector_sym_t res(sz);
  for (std::size_t i = 0; i < sz; ++i) {
    res[i] = sym_t::sym(name + std::to_string(i));
  }
  return res;
}

namespace casadi {
::casadi::SX eigen_to_casadi(const Eigen::MatrixX<::casadi::SX> &v) {
  ::casadi::SX c(v.rows(), v.cols());
  for (Eigen::Index i = 0; i < v.rows(); ++i) {
    for (Eigen::Index j = 0; j < v.cols(); ++j) {
      // Only fill in non-zero entries
      if (!::casadi::is_zero(v(i, j)->at(0))) {
        c(i, j) = v(i, j)->at(0);
      }
    }
  }
  return c;
}

Eigen::MatrixX<::casadi::SX> casadi_to_eigen(const ::casadi::SX &c) {
  Eigen::MatrixX<::casadi::SX> e(c.rows(), c.columns());
  for (index_t i = 0; i < c.rows(); ++i) {
    for (index_t j = 0; j < c.columns(); ++j) {
      e(i, j) = c(i, j);
    }
  }
  return e;
}

}  // namespace casadi

}  // namespace osc