#pragma once

#include "osc/common.hpp"

namespace osc {

// Forward declaration
struct SE3TrajectoryReference;
struct TrajectoryReference;

struct TrajectoryReference {
  TrajectoryReference(const SE3TrajectoryReference &ref) {
    position.resize(9);
    velocity.resize(6);
    acceleration.resize(6);
    // position << ref.position.position(), ref.position.rotation().asVector()
    velocity << ref.velocity.linear(), ref.velocity.angular();
    acceleration << ref.acceleration.linear(), ref.acceleration.angular();
  }
};

struct SE3TrajectoryReference {
  pinocchio::SE3 position;
  pinocchio::Motion velocity;
  pinocchio::Motion acceleration;

  SE3TrajectoryReference(const TrajectoryReference &ref) {}

  void set(const TrajectoryReference &ref) {
    // SE3
    position.translation() << ref.position.topRows(3);
    position.rotation() << ref.position.bottomRows(9);
    // Twist
    velocity.linear() = ref.velocity.topRows(3);
    velocity.angular() = ref.velocity.bottomRows(3);
    // Acceleration
    acceleration.linear() = ref.acceleration.topRows(3);
    acceleration.angular() = ref.acceleration.bottomRows(3);
  }
};

}  // namespace osc
