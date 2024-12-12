#include "osc/tasks/motion.hpp"

std::ostream &operator<<(std::ostream &os, osc::MotionTask const &m) {
  return os << "Task: " << m.name() << '\n'
            << "e : " << m.get_error().transpose() << '\n'
            << "e_dot : " << m.get_error_dot().transpose() << '\n'
            << "Kp : " << m.Kp().transpose() << '\n'
            << "Kd : " << m.Kd().transpose() << '\n'
            << "desired acceleration : " << m.get_desired_acceleration().transpose() << '\n';
}
