#pragma once

#include "osc/common.hpp"

namespace osc {

// Forward declaration
struct SE3TrajectoryReference;
struct TrajectoryReference;

struct TrajectoryReference {
  TrajectoryReference() = default;

  TrajectoryReference(const index_t &npos, const index_t &nvel,
                      const index_t &nacc);

  TrajectoryReference(const SE3TrajectoryReference &ref);

  void set(const SE3TrajectoryReference &ref);

  vector_t position;
  vector_t velocity;
  vector_t acceleration;
};

std::ostream &operator<<(std::ostream &os, TrajectoryReference const &r);

struct SE3TrajectoryReference {
  pinocchio::SE3 position;
  pinocchio::Motion velocity;
  pinocchio::Motion acceleration;

  SE3TrajectoryReference() {
    position = pinocchio::SE3::Identity();
    velocity = pinocchio::Motion::Zero();
    acceleration = pinocchio::Motion::Zero();
  }
  
  SE3TrajectoryReference(const TrajectoryReference &ref) {}

  void set(const TrajectoryReference &ref);
};

std::ostream &operator<<(std::ostream &os, SE3TrajectoryReference const &r);

}  // namespace osc
