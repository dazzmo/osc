#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace osc {
typedef ::casadi::SX sym_t;

typedef pinocchio::ModelTpl<sym_t> model_sym_t;
typedef pinocchio::DataTpl<sym_t> data_sym_t;

template<typename T>
using Eigen::VectorX<T> eigen_vector_tpl_t;

typedef eigen_vector_tpl_t<sym_t> eigen_vector_sym_t;
typedef eigen_vector_tpl_t<double> eigen_vector_t;

typedef std::shared_ptr<model_sym_t> model_sym_shared_ptr_t;

}  // namespace osc