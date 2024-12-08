/**
 * @file osc.hpp
 * @author your name (you@domain.com)
 * @brief Weighted Operational Space Control
 * @version 0.1
 * @date 2024-11-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "osc/common.hpp"
#include "osc/contacts/contact.hpp"
#include "osc/dynamics.hpp"
#include "osc/tasks/actuation.hpp"
#include "osc/tasks/motion.hpp"
#include "osc/qp_data.hpp"

namespace osc {



/**
 * @brief Default Operational Space Controller
 *
 */
class DefaultFormulation {
 public:
  DefaultFormulation(const model_t &model, const index_t &nq, const index_t &nv,
                     const index_t &nu);

  const index_t &n_variables() const { return nv_; }
  const index_t &n_eq_constraints() const { return nceq_; }
  const index_t &n_in_constraints() const { return ncin_; }

  /**
   * @brief Add a motion task to the program
   *
   * @param task
   * @param priority
   * @param weighting
   */
  void add_motion_task(const std::shared_ptr<MotionTask> &task,
                       const index_t &priority, const double &weighting = 1.0) {
    motion_tasks_.push_back(TaskInfo<MotionTask>(task, priority, weighting));
  }

  /**
   * @brief Removes a motion task from the program
   *
   * @param name
   * @return true Task was found and removed
   * @return false Task was not found
   */
  bool remove_motion_task(const string_t &name) {
    bool found = false;
    for (auto it = motion_tasks_.begin(); it != motion_tasks_.end(); ++it) {
      if (it->task->name() == name) {
        motion_tasks_.erase(it);
        found = true;
      }
    }
    return found;
  }

  /**
   * @brief Add an actuation task to the program
   *
   * @param task
   * @param priority
   * @param weighting
   */
  void add_actuation_task(const std::shared_ptr<ActuationTask> &task,
                          const index_t &priority,
                          const double &weighting = 1.0) {
    actuation_tasks_.push_back(
        TaskInfo<ActuationTask>(task, priority, weighting));
  }

  /**
   * @brief
   *
   * @param name
   * @return true
   * @return false
   */
  bool remove_actuation_task(const string_t &name) {
    bool found = false;
    for (auto it = actuation_tasks_.begin(); it != actuation_tasks_.end();
         ++it) {
      if (it->task->name() == name) {
        actuation_tasks_.erase(it);
        found = true;
      }
    }
    return found;
  }

  // void add_bounds(const std::shared_ptr<AccelerationBounds> &bounds);
  // void add_bounds(const std::shared_ptr<ActuationBounds> &bounds);

  void add_contact(const std::shared_ptr<AbstractFrictionContact> &contact,
                   const index_t &priority, const double &weighting = 1.0) {
    // todo - check that contact isn't currently being removed
    ContactInfo info;
    nk_ += contact->dim();
  }

  /**
   * @brief Removes a contact from the problem, providing a slow release of the
   * contact for program stability.
   *
   * @param name
   * @param t
   * @param duration
   */
  void remove_contact(const string_t &name, const double &t,
                      const double &duration) {
    // todo - check that contact isn't already being removed
    // Find the associated contact
    std::shared_ptr<AbstractFrictionContact> contact = nullptr;
    for (auto it = contacts_.begin(); it != contacts_.end(); ++it) {
      if (it->contact->name() == name) {
        contact = it->contact;
      }
    }
    // If the contact was found
    if (contact) {
      contact_releases_.push_back(ContactReleaseInfo(contact, t, duration));
    }
  }

  /**
   * @brief Computes all tasks and constraints and manages contact handling for
   * the current time step. Use before `qp_data()` to ensure correct problem
   * dimensions are given.
   *
   * @param t
   * @param q
   * @param v
   */
  void compute(const double &t, const vector_t &q, const vector_t &v);

  void set_qp_data(QuadraticProgramData &qp_data);

  void add_dynamics(const std::shared_ptr<InverseDynamics> &dynamics) {
    dynamics_provided_ = true;
    dynamics_ = dynamics;
  }

 protected:
  // Number of generalised acceleration variables
  index_t na_;
  // Number of actuation variables
  index_t nu_;
  // Number of constraint force decision variables
  index_t nk_;

  index_t nv_;
  index_t ncin_;
  index_t nceq_;

  template <typename TaskType>
  struct TaskInfo {
    TaskInfo(const std::shared_ptr<TaskType> &task, const index_t &priority,
             const double &weighting) {
      this->task = task;
      this->priority = priority;
      this->weighting = weighting;
    }

    std::shared_ptr<TaskType> task;
    // Task priority
    index_t priority = 0;
    // Task weighting
    double weighting = 1.0;
  };

  struct ContactInfo {
    std::shared_ptr<AbstractFrictionContact> contact;
    // Contact Priority
    index_t priority = 0;
    // Weighting of the contact
    double weighting = 1.0;
    // Index within the decision variables
    index_t index = 0;
  };

  struct ContactReleaseInfo {
    ContactReleaseInfo(const std::shared_ptr<AbstractFrictionContact> &contact,
                       const double &t, const double &duration) {
      this->contact = contact;
      t0 = t;
      tf = t0 + duration;
      f_max = contact->get_max_normal_force();
    }

    // Associated contact
    std::shared_ptr<AbstractFrictionContact> contact;
    // Starting time of release
    double t0;
    // Expected release time
    double tf;
    // Maximum allowable contact force when asked to release
    double f_max;
  };

  std::vector<TaskInfo<MotionTask>> motion_tasks_;
  std::vector<TaskInfo<ActuationTask>> actuation_tasks_;

  std::vector<ContactInfo> contacts_;
  std::vector<ContactReleaseInfo> contact_releases_;

  std::shared_ptr<InverseDynamics> dynamics_;

 private:
  // Model
  model_t model;
  // Flag to indicate if dynamics have been provided
  bool dynamics_provided_;
};

}  // namespace osc
