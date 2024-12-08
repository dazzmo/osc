#pragma once

#include <qpOASES/QProblem.hpp>
#include <qpOASES/SQProblem.hpp>

#include "osc/common.hpp"
#include "osc/qp_data.hpp"

namespace osc {

class qpOASESSolver {
 public:
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      row_major_matrix_t;
  typedef Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      row_major_matrix_map_t;

  qpOASESSolver(const index_t &nv, const index_t &nceq, const index_t &ncin) {
    A_ = row_major_matrix_t::Zero(nceq + ncin, nv);
    b_ = vector_t::Zero(nceq + ncin);

    lbA_ = vector_t::Constant(nceq + ncin, -qpOASES::INFTY);
    lbA_.topRows(nceq).setZero();
    ubA_ = vector_t::Zero(nceq + ncin);
  }

  vector_t get_solution() const {
    vector_t x(nv_);
    qp_->getPrimalSolution(x.data());
    return x;
  }

  const qpOASES::QProblemStatus &get_status() { qp_->getStatus(); }

  void solve(const QuadraticProgramData &data, bool hotstart = false,
             const int &nWSR = 200) {
    // Combine equality and inequality constraints
    A_.topRows(data.Aeq.rows()) << data.Aeq;
    A_.bottomRows(data.Ain.rows()) << data.Ain;

    b_.topRows(data.Aeq.rows()) << data.beq;
    b_.bottomRows(data.Ain.rows()) << data.bin;

    row_major_matrix_map_t H(data.H.data, nv, nv), A();

    // Solve
    if (hotstart) {
      profiler("qpoases_solver");
      // Use previous solution to hot-start the program
      qp_->hotstart(H.data(), data.g.data(), A_.data(), data.lbx.data(),
                    data.ubx.data(), lbA_.data(), ubA_.data(), nWSR);
    } else {
      profiler("qpoases_solver");
      // Initialise the program and solve it
      qp_->init(H.data(), data.g.data(), A_.data(), data.lbx.data(),
                data.ubx.data(), lbA_.data(), ubA_.data(), nWSR);
    }
  }

  void reset() { qp_->reset(); }

 private:
  std::unique_ptr<qpOASES::SQProblem> qp_;

  row_major_matrix_t A_;
  vector_t b_;
  vector_t lbA_;
  vector_t ubA_;
};

}  // namespace osc