#include "osc/trajectory_reference.hpp"

namespace osc {

TrajectoryReference::TrajectoryReference(const index_t &npos,
                                         const index_t &nvel,
                                         const index_t &nacc) {
  position = vector_t::Zero(npos);
  velocity = vector_t::Zero(nvel);
  acceleration = vector_t::Zero(nacc);
}

TrajectoryReference::TrajectoryReference(const SE3TrajectoryReference &ref) {
  position = vector_t::Zero(9);
  velocity = vector_t::Zero(6);
  acceleration = vector_t::Zero(6);
}

void TrajectoryReference::set(const SE3TrajectoryReference &ref) {
  position << ref.position.translation(),
      Eigen::Map<const vector_t>(ref.position.rotation().data(), 9);
  velocity << ref.velocity.linear(), ref.velocity.angular();
  acceleration << ref.acceleration.linear(), ref.acceleration.angular();
}

std::ostream &operator<<(std::ostream &os, TrajectoryReference const &r) {
  return os << "position : " << r.position.transpose() << '\n'
            << "velocity : " << r.velocity.transpose() << '\n'
            << "acceleration : " << r.acceleration.transpose() << '\n';
}

void SE3TrajectoryReference::set(const TrajectoryReference &ref) {
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

std::ostream &operator<<(std::ostream &os, SE3TrajectoryReference const &r) {
  return os << "position : " << r.position << '\n'
            << "velocity : " << r.velocity << '\n'
            << "acceleration : " << r.acceleration << '\n';
}

}  // namespace osc