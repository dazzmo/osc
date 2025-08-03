#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace osc {

// Core types
using String = std::string;
using Real = double;
using Integer = long long int;
using Size = std::size_t;
using Index = Eigen::Index;

// Pinocchio types
using Model = pinocchio::ModelTpl<Real>;
using ModelData = pinocchio::DataTpl<Real>;

// using SymbolicModel = pinocchio::ModelTpl<casadi::SX>;
// using SymbolicModelData = pinocchio::DataTpl<casadi::SX>;

using SE3 = pinocchio::SE3Tpl<Real>;
using Motion = pinocchio::MotionTpl<Real>;

// Linear Algebra
template <typename T>
using VectorX = Eigen::VectorX<T>;
template <typename T>
using MatrixX = Eigen::MatrixX<T>;

using Vector = VectorX<Real>;
using Matrix = MatrixX<Real>;

using Vector3 = Eigen::Vector<Real, 3>;
using Matrix3 = Eigen::Matrix<Real, 3, 3>;

using Vector6 = Eigen::Vector<Real, 6>;
using Matrix6 = Eigen::Matrix<Real, 6, 6>;

}  // namespace osc