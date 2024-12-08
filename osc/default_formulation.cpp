#include "osc/default_formulation.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace osc {

DefaultFormulation::DefaultFormulation(const model_t &model, const index_t &nq,
                                       const index_t &nv, const index_t &nu)
    : na_(nv),
      nu_(nu),
      nk_(0),
      nv_(nv + nu),
      model(model),
      actuation_bounds_(nullptr),
      acceleration_bounds_(nullptr),
      dynamics_provided_(false) {}

void DefaultFormulation::compute(const double &t, const vector_t &q,
                                 const vector_t &v) {
  assert(dynamics_provided_ && "System dynamics not provided");
  // Update model data
  data_t data(model);
  // Update pinocchio model for computations
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::centerOfMass(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  // Check if any scheduled contact points are to be removed from the program,
  // if not, continue decreasing their maximum normal reaction forces.
  for (auto &info : contact_releases_) {
    if (t <= info.tf) {
      // Linearly scale the maximum normal force to zero for the contact
      double n_max_new = info.f_max * (info.tf - t) / (info.tf - info.t0);
      info.contact->set_max_normal_force(n_max_new);
    } else {
      // Remove contact from the contact vector and variables
      nk_ -= info.contact->dim();
      nv_ -= info.contact->dim();
      ncin_ -= 4;
      // Remove from the vector
      for (auto it = contacts_.begin(); it != contacts_.end(); it++) {
        if (it->contact->name() == info.contact->name()) {
          // Remove this entry and update all indices
          contacts_.erase(it);
        }
      }
    }
  }

  // Update contact indices
  int idx = 0;
  for (auto &contact : contacts_) {
    contact.index = idx;
    idx += contact.contact->dim();
  }

  // Motion tasks
  for (auto &info : motion_tasks_) {
    info.task->compute(model, data, q, v);
  }

  // Contact
  for (auto &info : contacts_) {
    info.contact->compute(model, data, q, v);
  }

  // Dynamics constraints
  dynamics_->compute(model, data, q, v);
}

void DefaultFormulation::set_qp_data(QuadraticProgramData &qp_data) {
  int i_eq = 0;
  int i_in = 0;

  for (auto &info : motion_tasks_) {
    // If priority is 0, make it a constraint
    const index_t &priority = info.priority;
    const double &w = info.weighting;
    auto &task = info.task;

    const matrix_t &J = task->jacobian();
    const matrix_t &dJdq = task->jacobian_dot_q_dot();
    const matrix_t &xacc_d = task->get_desired_acceleration();

    if (priority == 0) {
      // J \ddot q + \dot J \dot q = \ddot x_d
      qp_data.Aeq.middleRows(i_eq, na_) = J;
      qp_data.beq.middleRows(i_eq, task->dim()) = dJdq - xacc_d;
    } else {
      // || J \ddot q + \dot J \dot q - \ddot x_d ||^2
      qp_data.H.topRightCorner(na_, na_) += w * J.transpose() * J;
      qp_data.g.topRows(na_) += 2.0 * w * (J.transpose() * (dJdq - xacc_d));
    }
  }

  // Contacts
  for (auto &info : contacts_) {
    const index_t &priority = info.priority;
    const double &w = info.weighting;
    auto &contact = info.contact;

    const matrix_t &J = contact->jacobian();
    const matrix_t &dJdq = contact->jacobian_dot_q_dot();

    if (priority == 0) {
      // J \ddot q + \dot J \dot q = 0
      qp_data.Aeq.middleRows(i_eq, na_) = J;
      qp_data.beq.middleRows(i_eq, contact->dim()) = dJdq;
      i_eq += contact->dim();
    } else {
      // || J \ddot q + \dot J \dot q - \ddot x_d ||^2
      qp_data.H.topRightCorner(na_, na_) += J.transpose() * J;
      qp_data.g.topRows(na_) += 2.0 * (J.transpose() * (dJdq));
    }

    // Add friction constraint
    int idx = info.index;
    auto constraint = contact->friction_cone_constraint();
    qp_data.Ain.middleRows(i_in, 4).middleCols(na_ + nu_ + idx,
                                               contact->dim()) = constraint.A();
    qp_data.bin.middleRows(i_in, 4) = constraint.b();

    // Update bounds for normal force
    qp_data.x_lb[na_ + nu_ + (idx + 2)] = contact->get_min_normal_force();
    qp_data.x_ub[na_ + nu_ + (idx + 2)] = contact->get_max_normal_force();
  }

  // Dynamics constraints
  const matrix_t &M = dynamics_->get_inertial_matrix();
  const matrix_t &Minv = dynamics_->get_inertial_matrix_inverse();
  const matrix_t &h = dynamics_->get_nonlinear_terms();
  const matrix_t &B = dynamics_->get_actuation_map();

  if (dynamics_->dim_constraints() > 0) {
    const matrix_t &Jc = dynamics_->get_holonomic_constraint_jacobian();
    const vector_t &dJcdq =
        dynamics_->get_holonomic_constraint_jacobian_dot_q_dot();
    // Set constraint-projected dynamics
    matrix_t N = matrix_t::Identity(na_, na_);
    matrix_t Lambda = (Jc * Minv * Jc.transpose())
                          .completeOrthogonalDecomposition()
                          .pseudoInverse();
    N -= Jc.transpose() * Lambda * Jc * Minv;

    // Inertial matrix
    qp_data.Aeq.middleRows(i_eq, na_).leftCols(na_) = M;
    // Contact jacobians
    for (auto &contact : contacts_) {
      qp_data.Aeq.middleRows(i_eq, na_).middleCols(na_ + nu_ + contact.index,
                                                   contact.contact->dim()) =
          -N * contact.contact->jacobian().transpose();
    }
    // Actuation matrix
    qp_data.Aeq.middleRows(i_eq, na_).middleCols(na_, nu_) = -N * B;
    // Bias vector
    qp_data.beq.middleRows(i_eq, na_) = -N * h + Jc * Lambda * dJcdq;
  } else {
    // Inertial matrix
    qp_data.Aeq.middleRows(i_eq, na_).leftCols(na_) = M;
    // Contact jacobians
    for (auto &contact : contacts_) {
      qp_data.Aeq.middleRows(i_eq, na_).middleCols(na_ + nu_ + contact.index,
                                                   contact.contact->dim()) =
          -contact.contact->jacobian().transpose();
    }
    // Actuation matrix
    qp_data.Aeq.middleRows(i_eq, na_).middleCols(na_, nu_) = -B;
    // Bias vector
    qp_data.beq.middleRows(i_eq, na_) = -h;
  }

  // Bounds
  if (acceleration_bounds_) {
    qp_data.x_lb.topRows(na_) = acceleration_bounds_->lower_bound();
    qp_data.x_ub.topRows(na_) = acceleration_bounds_->upper_bound();
  }

  if (actuation_bounds_) {
    qp_data.x_lb.middleRows(na_, nu_) = actuation_bounds_->lower_bound();
    qp_data.x_ub.middleRows(na_, nu_) = actuation_bounds_->upper_bound();
  }
}

}  // namespace osc