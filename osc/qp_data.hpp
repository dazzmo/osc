#pragma once

#include "osc/common.hpp"

namespace osc {

/**
 * @brief Quadratic program data of the form
 * min 0.5 x^T Q x + g^T x
 * s.t. Aeq x + beq = 0
 *      Ain x + bin <= 0
 *      x_lb <= x <= x_ub
 *
 */
struct QuadraticProgramData {
  /**
   * @brief Construct a new Quadratic Program Data
   *
   * @param nx Number of decision variables in the program.
   * @param neq Number of equality constraints in the program.
   * @param nin Number of inequality constraints in the program.
   */
  QuadraticProgramData(const index_t &nx, const index_t &neq,
                       const index_t &nin) {
    H = matrix_t::Zero(nx, nx);
    g = vector_t::Zero(nx);

    Aeq = matrix_t::Zero(neq, nx);
    Ain = matrix_t::Zero(nin, nx);

    beq = vector_t::Zero(neq);
    bin = vector_t::Zero(nin);

    x_lb = vector_t::Zero(nx);
    x_ub = vector_t::Zero(nx);
  }

  // QP Hessian matrix
  matrix_t H;
  // QP gradient vector
  vector_t g;

  // Equality constraint information of the form A x + b = 0
  matrix_t Aeq;
  vector_t beq;

  // Inequality constraint information of the form A x + b <= 0
  matrix_t Ain;
  matrix_t bin;

  // Bounds
  vector_t x_ub;
  vector_t x_lb;
};

}  // namespace osc
