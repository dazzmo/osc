#pragma once
#include "osc/common.hpp"
namespace osc {

/**
 * @brief Linear constraint of the form lower_bound() <= A() x + b() <=
 * upper_bound()
 *
 */
class LinearConstraint {
 public:
  enum Type { Equality = 0, Inequality, Bounds };

  LinearConstraint(const index_t &nc, const index_t &nv,
                   const &type = Type::Equality)
      : type_(type) {
    A_ = matrix_t::Zero(nc, nv);
    b_ = vector_t::Zero(nc);

    lower_bound_ = vector_t::Constant(nc, -std::numeric_limits<double>::max());
    upper_bound_ = vector_t::Constant(nc, std::numeric_limits<double>::max());
  }

  const Type &type() const { return type_; }

  const matrix_t &A() const { return A_; }
  matrix_t &A() { return A_; }

  const matrix_t &b() const { return b_; }
  matrix_t &b() { return b_; }

  const vector_t &upper_bound() const { return upper_bound_; }
  vector_t &upper_bound() { return upper_bound_; }

  const vector_t &lower_bound() const { return lower_bound_; }
  vector_t &lower_bound() { return lower_bound_; }

 private:
  Type type_;

  matrix_t A_;
  matrix_t b_;

  vector_t lower_bound_;
  vector_t upper_bound_;
};

}  // namespace osc