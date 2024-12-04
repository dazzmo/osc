#pragma once

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>
#include <bopt/variable.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace osc {

typedef std::string string_t;
typedef std::size_t index_t;

typedef ::casadi::SXElem sym_elem_t;
typedef ::casadi::SX sym_t;
typedef std::vector<sym_t> sym_vector_t;

typedef pinocchio::ModelTpl<sym_t> model_sym_t;
typedef pinocchio::DataTpl<sym_t> data_sym_t;

typedef pinocchio::Model model_t;
typedef pinocchio::Data data_t;

template <typename T, int Size = Eigen::Dynamic>
using vector_tpl_t = Eigen::Vector<T, Size>;

template <typename T, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
using matrix_tpl_t = Eigen::Matrix<T, Rows, Cols>;

typedef vector_tpl_t<bopt::variable> vector_var_t;
typedef vector_tpl_t<sym_t> vector_sym_t;
typedef vector_tpl_t<double> vector_t;
typedef vector_tpl_t<double, 3> vector3_t;

typedef matrix_tpl_t<sym_t> matrix_sym_t;
typedef matrix_tpl_t<double> matrix_t;
typedef matrix_tpl_t<double, 3, 3> matrix3_t;

vector_sym_t create_symbolic_vector(const string_t &name, const index_t &sz);

/**
 * @brief Variables associated with an Operational Space controller
 *
 * @tparam Scalar
 */
struct osc_variables {
  std::vector<bopt::variable> a;
  std::vector<bopt::variable> u;
  std::vector<bopt::variable> lambda;
};

/**
 * @brief Generic parameters associated with an operational space controller
 *
 * @tparam Scalar
 */
struct osc_parameters {
  std::vector<bopt::variable> q;
  std::vector<bopt::variable> v;
};

namespace casadi {

::casadi::SX eigen_to_casadi(const Eigen::MatrixX<::casadi::SX> &v);
Eigen::MatrixX<::casadi::SX> casadi_to_eigen(const ::casadi::SX &c);

}  // namespace casadi

}  // namespace osc