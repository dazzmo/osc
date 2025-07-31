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

using SE3 = pinocchio::SE3Tpl<Real>;

// Linear Algebra
template <typename T>
using VectorX = Eigen::VectorX<T>;
template <typename T>
using MatrixX = Eigen::MatrixX<T>;

using Vector = VectorX<Real>;
using Matrix = MatrixX<Real>;

}  // namespace osc