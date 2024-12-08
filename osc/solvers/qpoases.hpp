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
      const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      row_major_matrix_map_t;

  qpOASESSolver(const index_t &nv, const index_t &nceq, const index_t &ncin);

  void get_solution(vector_t &x) const ;

  const qpOASES::QProblemStatus &get_status();

  void solve(const QuadraticProgramData &data, const bool &hotstart = false,
             const int &nWSR = 200) ;

  void reset();

 private:
  index_t nv_;
  index_t nc_;

  std::unique_ptr<qpOASES::SQProblem> qp_;

  row_major_matrix_t A_;
  vector_t b_;
  vector_t lbA_;
  vector_t ubA_;
};

}  // namespace osc