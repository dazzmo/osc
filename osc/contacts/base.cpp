#include "osc/contacts/base.hpp"

std::ostream &operator<<(std::ostream &os, osc::ContactBase const &c) {
  return os << "Contact: " << c.name() << '\n'
            << "reference frame : " << c.reference_frame() << '\n'
            << "e : " << c.get_error().transpose() << '\n'
            << "e_dot : " << c.get_error_dot().transpose() << '\n'
            << "surface_normal : " << c.get_surface_normal().transpose() << '\n'
            << "friction_coefficient : " << c.get_friction_coefficient() << '\n'
            << "min_normal_force : " << c.get_min_normal_force() << '\n'
            << "max_normal_force : " << c.get_max_normal_force() << '\n'
            << "Kp : " << c.Kp().transpose() << '\n'
            << "Kd : " << c.Kd().transpose() << '\n'
            << "reference : " << c.get_reference().position << '\n';
}