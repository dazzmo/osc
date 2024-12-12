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
      ncin_(0),
      nceq_(0),
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
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::centerOfMass(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  VLOG(10) << "Check contact removals";
  // Check if any scheduled contact points are to be removed from the program,
  // if not, continue decreasing their maximum normal reaction forces.
  for (auto info_it = contact_releases_.begin();
       info_it != contact_releases_.end();) {
    VLOG(10) << "name: " << info_it->contact->name();
    bool removed = false;
    if (t <= info_it->tf) {
      VLOG(10) << "Scaling";
      // Linearly scale the maximum normal force to zero for the contact
      double n_max_new =
          info_it->f_max * (info_it->tf - t) / (info_it->tf - info_it->t0);
      info_it->contact->set_max_normal_force(n_max_new);
    } else {
      VLOG(10) << "Removing";
      // Remove contact from the contact vector and variables
      nk_ -= info_it->contact->dim();
      nv_ -= info_it->contact->dim();
      // todo - check if this is a priority 0 contact
      // nceq_ -= info.contact->dim();
      ncin_ -= 6;
      // Remove from the vector
      for (auto it = contacts_.begin(); it != contacts_.end();) {
        if (it->contact->name() == info_it->contact->name()) {
          it = contacts_.erase(it);
          removed = true;
        } else {
          it++;
        }
      }
    }
    if (removed) {
      info_it = contact_releases_.erase(info_it);
    } else {
      info_it++;
    }
  }

  VLOG(10) << "Update indices";
  // Update contact indices
  int idx = 0;
  for (auto &contact : contacts_) {
    contact.index = idx;
    idx += contact.contact->dim();
  }

  VLOG(10) << "Motion";
  // Motion tasks
  for (auto &info : motion_tasks_) {
    info.task->compute(model, data, q, v);
  }

  VLOG(10) << "Contact";
  // Contact
  for (auto &info : contacts_) {
    info.contact->compute(model, data, q, v);
  }

  VLOG(10) << "Dynamics";
  // Dynamics constraints
  dynamics_->compute(model, data, q, v);
}

void DefaultFormulation::set_qp_data(QuadraticProgramData &qp_data) {
  int i_eq = 0;
  int i_in = 0;

  qp_data.H.setZero();
  qp_data.g.setZero();

  // Motion tasks
  VLOG(10) << "Motion Tasks";
  for (auto &info : motion_tasks_) {
    // If priority is 0, make it a constraint
    const index_t &priority = info.priority;
    const double &w = info.weighting;
    auto &task = info.task;

    const matrix_t &J = task->jacobian();
    const vector_t &dJdq = task->jacobian_dot_q_dot();
    const vector_t &xacc_d = task->get_desired_acceleration();
    if (priority == 0) {
      // J \ddot q + \dot J \dot q = \ddot x_d
      qp_data.Aeq.middleRows(i_eq, task->dim()).leftCols(na_) = J;
      qp_data.beq.middleRows(i_eq, task->dim()) = dJdq - xacc_d;
      i_eq += task->dim();
    } else {
      // || J \ddot q + \dot J \dot q - \ddot x_d ||^2
      qp_data.H.topLeftCorner(na_, na_) += w * J.transpose() * J;
      qp_data.g.topRows(na_) += 2.0 * w * (J.transpose() * (dJdq - xacc_d));
    }
  }

  VLOG(10) << "Actuation Tasks";
  // Actuation tasks
  for (auto &info : actuation_tasks_) {
    // If priority is 0, make it a constraint
    const index_t &priority = info.priority;
    const double &w = info.weighting;
    auto &task = info.task;

    const matrix_t &J = task->jacobian();

    // || J u ||^2
    qp_data.H.block(na_, na_, nu_, nu_) += w * J.transpose() * J;
  }

  VLOG(10) << "Contacts";
  // Contacts
  for (auto &info : contacts_) {
    const index_t &priority = info.priority;
    const double &w = info.weighting;
    auto &contact = info.contact;

    const matrix_t &J = contact->jacobian();
    const matrix_t &dJdq = contact->jacobian_dot_q_dot();
    const vector_t &xacc_d = contact->get_desired_acceleration();

    if (priority == 0) {
      // J \ddot q + \dot J \dot q = \ddot x_d
      qp_data.Aeq.middleRows(i_eq, contact->dim()).leftCols(na_) = J;
      qp_data.beq.middleRows(i_eq, contact->dim()) = dJdq - xacc_d;
      i_eq += contact->dim();
    } else {
      // || J \ddot q + \dot J \dot q - \ddot x_d ||^2
      qp_data.H.topLeftCorner(na_, na_) += w * J.transpose() * J;
      qp_data.g.topRows(na_) += 2.0 * w * (J.transpose() * (dJdq - xacc_d));
    }

    // Add friction constraint
    int idx = info.index;
    auto constraint = contact->friction_cone_constraint();
    qp_data.Ain.middleRows(i_in, 6).middleCols(na_ + nu_ + idx,
                                               contact->dim()) = constraint.A();
    qp_data.bin.middleRows(i_in, 6) = constraint.b();

    i_in += 6;
  }

  VLOG(10) << "Dynamics";
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
    matrix_t Lambda =
        (Jc * Minv.selfadjointView<Eigen::Upper>() * Jc.transpose())
            .completeOrthogonalDecomposition()
            .pseudoInverse();
    N -= Jc.transpose() * Lambda * Jc * Minv.selfadjointView<Eigen::Upper>();

    // Inertial matrix
    qp_data.Aeq.middleRows(i_eq, na_).leftCols(na_) =
        M.selfadjointView<Eigen::Upper>();
    // Contact jacobians
    for (auto &contact : contacts_) {
      qp_data.Aeq.middleRows(i_eq, na_).middleCols(na_ + nu_ + contact.index,
                                                   contact.contact->dim()) =
          -N * contact.contact->jacobian().transpose();
    }
    // Actuation matrix
    qp_data.Aeq.middleRows(i_eq, na_).middleCols(na_, nu_) = -N * B;
    // Bias vector
    qp_data.beq.middleRows(i_eq, na_) = N * h + Jc.transpose() * Lambda * dJcdq;

    i_eq += na_;

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
    qp_data.beq.middleRows(i_eq, na_) = h;

    i_eq += na_;
  }

  VLOG(10) << "Bounds";
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