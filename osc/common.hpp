#pragma once

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace osc {

typedef std::string string_t;
typedef std::size_t index_t;

typedef pinocchio::Model model_t;
typedef pinocchio::Data data_t;

template <typename T, int Size = Eigen::Dynamic>
using vector_tpl_t = Eigen::Vector<T, Size>;

template <typename T, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
using matrix_tpl_t = Eigen::Matrix<T, Rows, Cols>;

typedef vector_tpl_t<double> vector_t;
typedef vector_tpl_t<double, 3> vector3_t;

typedef matrix_tpl_t<double> matrix_t;
typedef matrix_tpl_t<double, 3, 3> matrix3_t;

}  // namespace osc