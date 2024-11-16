#pragma once

#include <Eigen/Core>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace osc {

typedef std::string string_t;

typedef ::casadi::SXElem sym_elem_t;
typedef ::casadi::SX sym_t;
typedef std::vector<sym_t> sym_vector_t;

typedef pinocchio::ModelTpl<sym_t> model_sym_t;
typedef pinocchio::DataTpl<sym_t> data_sym_t;

template <typename T>
using eigen_vector_tpl_t = Eigen::VectorX<T>;

template <typename T>
using eigen_matrix_tpl_t = Eigen::MatrixX<T>;

typedef eigen_vector_tpl_t<bopt::variable> eigen_vector_var_t;
typedef eigen_vector_tpl_t<sym_t> eigen_vector_sym_t;
typedef eigen_vector_tpl_t<double> eigen_vector_t;

typedef eigen_matrix_tpl_t<sym_t> eigen_matrix_sym_t;
typedef eigen_matrix_tpl_t<double> eigen_matrix_t;

typedef std::shared_ptr<model_sym_t> model_sym_shared_ptr_t;

eigen_vector_sym_t create_symbolic_vector(const std::string &name,
                                          const std::size_t &sz);

eigen_vector_var_t create_variable_vector(const std::string &name,
                                          const std::size_t &sz);

}  // namespace osc