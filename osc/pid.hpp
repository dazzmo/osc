#pragma once

#include "osc/common.hpp"

namespace osc {

template <typename T>
struct pid_error {
  pid_error(const index_t &n) : error(n), error_dot(n), error_int(n) {}
  // Error
  vector_tpl_t<T> error;
  // Time derivative of the error
  vector_tpl_t<T> error_dot;
  // Integral of the error
  vector_tpl_t<T> error_int;
};

template <typename T>
struct pid_gains {
  pid_gains(const std::size_t &sz) : p(sz), i(sz), d(sz) {}

  void resize(const std::size_t &sz) {
    p.resize(sz);
    i.resize(sz);
    d.resize(sz);
  }

  vector_tpl_t<T> p;
  vector_tpl_t<T> i;
  vector_tpl_t<T> d;
};

template<typename T>
struct pid {
  static constexpr vector_tpl_t<T> compute_error(const pid_gains<T> &gains,
                                               const pid_error<T> &error) {
    return gains.p.asDiagonal() * error.error +
           gains.i.asDiagonal() * error.error_int +
           gains.d.asDiagonal() * error.error_dot;
  }
};

}  // namespace osc
