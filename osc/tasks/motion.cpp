#pragma once
#include "osc/tasks/motion.hpp"

std::ostream &operator<<(std::ostream &os, osc::MotionTask const &m) {
  return os << "Task: " << m.name() << '\n'
            << "reference frame : " << m.reference_frame() << '\n'
            << "e : " << m.get_error().transpose() << '\n'
            << "e_dot : " << m.get_error_dot().transpose() << '\n'
            << "Kp : " << m.Kp().transpose() << '\n'
            << "Kd : " << m.Kd().transpose() << '\n';
}
