#include "osc/solvers/qpoases.hpp"

namespace osc {

qpOASESSolver::qpOASESSolver(const index_t &nv, const index_t &nceq,
                             const index_t &ncin)
    : nv_(nv), nc_(nceq + ncin), qp_(nullptr) {
  A_ = row_major_matrix_t::Zero(nc_, nv);
  b_ = vector_t::Zero(nc_);

  lbA_ = vector_t::Constant(nc_, -qpOASES::INFTY);
  lbA_.topRows(nceq).setZero();
  ubA_ = vector_t::Zero(nc_);

  qp_ = std::make_unique<qpOASES::SQProblem>(nv_, nc_,
                                             qpOASES::HessianType::HST_POSDEF);
}

void qpOASESSolver::get_solution(vector_t &x) const {
  qp_->getPrimalSolution(x.data());
}

const qpOASES::QProblemStatus &qpOASESSolver::get_status() { qp_->getStatus(); }

void qpOASESSolver::solve(const QuadraticProgramData &data,
                          const bool &hotstart, const int &nWSR) {
  // Combine equality and inequality constraints
  A_.topRows(data.Aeq.rows()) << data.Aeq;
  A_.bottomRows(data.Ain.rows()) << data.Ain;

  b_.topRows(data.Aeq.rows()) << data.beq;
  b_.bottomRows(data.Ain.rows()) << data.bin;

  row_major_matrix_map_t H(data.H.data(), nv_, nv_);

  // Solve
  int nWSR_ = nWSR;
  if (hotstart) {
    // Use previous solution to hot-start the program
    qp_->hotstart(H.data(), data.g.data(), A_.data(), data.x_lb.data(),
                  data.x_ub.data(), lbA_.data(), ubA_.data(), nWSR_);
  } else {
    // Initialise the program and solve it
    qp_->init(H.data(), data.g.data(), A_.data(), data.x_lb.data(),
              data.x_ub.data(), lbA_.data(), ubA_.data(), nWSR_);
  }
}

void qpOASESSolver::reset() { qp_->reset(); }

}  // namespace osc