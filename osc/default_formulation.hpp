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
#include "osc/contacts/base.hpp"
#include "osc/dynamics.hpp"
#include "osc/qp_data.hpp"
#include "osc/tasks/acceleration.hpp"
#include "osc/tasks/actuation.hpp"
#include "osc/tasks/motion.hpp"

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
   * @brief Adds a motion to the program `duration` seconds after the current
   * time `t`.
   *
   * @param task
   * @param priority
   * @param weighting
   * @param t
   * @param duration
   */
  void add_motion_task(const std::shared_ptr<MotionTask> &task,
                       const index_t &priority, const double &weighting,
                       const double &t, const double &duration = 0.0) {
    motion_tasks_scheduled_.push_back(
        {ScheduleTimer(ScheduleTimer::Type::ADD, t, duration),
         TaskInfo<MotionTask>(task, priority, weighting)});
  }

  void remove_motion_task(const string_t &name, const double &t,
                          const double &duration = 0.0) {
    // Find the associated task
    std::shared_ptr<MotionTask> task = nullptr;
    for (auto it = motion_tasks_.begin(); it != motion_tasks_.end(); ++it) {
      if (it->task->name() == name) {
        // If the contact is found, schedule for removal
        motion_tasks_scheduled_.push_back(
            {ScheduleTimer(ScheduleTimer::Type::REMOVE, t, duration), *it});
      }
    }
  }

  /**
   * @brief Add an actuation task to the program
   *
   * @param task
   * @param priority
   * @param weighting
   */
  void add_actuation_task(const std::shared_ptr<ActuationTask> &task,
                          const index_t &priority, const double &weighting,
                          const double &t, const double &duration = 0.0) {
    actuation_tasks_scheduled_.push_back(
        {ScheduleTimer(ScheduleTimer::Type::ADD, t, duration),
         TaskInfo<ActuationTask>(task, priority, weighting)});
  }

  void remove_actuation_task(const string_t &name, const double &t,
                             const double &duration = 0.0) {
    // Find the associated task
    std::shared_ptr<ActuationTask> task = nullptr;
    for (auto it = actuation_tasks_.begin(); it != actuation_tasks_.end();
         ++it) {
      if (it->task->name() == name) {
        // If the contact is found, schedule for removal
        actuation_tasks_scheduled_.push_back(
            {ScheduleTimer(ScheduleTimer::Type::REMOVE, t, duration), *it});
      }
    }
  }

  void add_contact(const std::shared_ptr<ContactBase> &contact,
                   const index_t &priority, const double &weighting,
                   const double &t, const double &duration = 0.0) {
    contacts_scheduled_.push_back(
        {ScheduleTimer(ScheduleTimer::Type::ADD, t, duration),
         ContactInfo(contact, priority, weighting)});
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
    // Find the associated contact
    std::shared_ptr<ContactBase> contact = nullptr;
    for (auto it = contacts_.begin(); it != contacts_.end(); ++it) {
      if (it->contact->name() == name) {
        // If the contact is found, schedule for removal, recording current max
        // normal force allowed for linear scaling
        it->f_max = it->contact->get_max_normal_force();
        contacts_scheduled_.push_back(
            {ScheduleTimer(ScheduleTimer::Type::REMOVE, t, duration), *it});
      }
    }
  }

  void add_bounds(const std::shared_ptr<AccelerationBounds> &bounds) {
    acceleration_bounds_ = bounds;
  }

  void add_bounds(const std::shared_ptr<ActuationBounds> &bounds) {
    actuation_bounds_ = bounds;
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

  /**
   * @brief Returns the actuation inputs associated with the program
   * from the solution vector `sol`.
   *
   * @param sol
   */
  vector_t get_actuation(const vector_t &sol) {
    return sol.middleRows(na_, nu_);
  }

  /**
   * @brief Returns the generalised accelerations  associated with the program
   * from the solution vector `sol`.
   *
   * @param sol
   */
  vector_t get_acceleration(const vector_t &sol) { return sol.topRows(na_); }

  /**
   * @brief Updates all contact forces associated with the program from the
   * solution vector `sol`.
   *
   * @param sol
   */
  void update_contact_forces(const vector_t &sol) {}

  void set_qp_data(QuadraticProgramData &qp_data);

  void add_dynamics(const std::shared_ptr<InverseDynamics> &dynamics) {
    dynamics_provided_ = true;
    dynamics_ = dynamics;
    nceq_ += dynamics->nv();
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

  // Schedule
  struct ScheduleTimer {
    // Addition or removal of a given object at the specified time
    enum class Type { ADD = 0, REMOVE };

    ScheduleTimer(const Type &type, const double &now,
                  const double &duration = 0.0) {
      type_ = type;
      t0_ = now;
      tf_ = now + duration;
    }

    /**
     * @brief Whether the required duration has elapsed
     *
     * @param now
     * @return true
     * @return false
     */
    bool act(const double &now) const { return now >= tf_; }

    const Type &type() const { return type_; }
    const double &t0() const { return t0_; }
    const double &tf() const { return tf_; }
    double duration() const { return tf_ - t0_; }

    double t0_;
    double tf_;
    Type type_;
  };

  struct TaskProperties {
    index_t priority = 0;
    double weighting = 1.0;
  };

  template <typename TaskType>
  struct TaskInfo : public TaskProperties {
    TaskInfo(const std::shared_ptr<TaskType> &task, const index_t &priority,
             const double &weighting) {
      this->task = task;
      this->priority = priority;
      this->weighting = weighting;
    }
    std::shared_ptr<TaskType> task;
  };

  struct ContactInfo : public TaskProperties {
    ContactInfo(const std::shared_ptr<ContactBase> &contact,
                const index_t &priority, const double &weighting) {
      this->contact = contact;
      this->priority = priority;
      this->weighting = weighting;
    }
    std::shared_ptr<ContactBase> contact;
    // Maximum normal force
    double f_max = 0.0;
    // Index within the decision variables
    index_t index = 0;
  };

  void check_scheduled_contacts(
      const double &t,
      std::vector<std::pair<ScheduleTimer, ContactInfo>> &schedule,
      std::vector<ContactInfo> &contacts) {
    // Assess contact schedule
    for (auto it = schedule.begin(); it != schedule.end();) {
      bool complete = false;

      const ScheduleTimer &timer = it->first;
      const ContactInfo &info = it->second;

      if (timer.act(t)) {
        if (timer.type() == ScheduleTimer::Type::REMOVE) {
          // Remove variables
          nk_ -= info.contact->dim();
          nv_ -= info.contact->dim();
          // Remove friction-cone constraint
          ncin_ -= 6;
          if (info.priority == 0) {
            nceq_ -= info.contact->dim();
          }
          // Remove from the vector
          for (auto jt = contacts.begin(); jt != contacts.end();) {
            if (jt->contact->name() == info.contact->name()) {
              jt = contacts.erase(jt);
            } else {
              jt++;
            }
          }

        } else {
          // Adding contact
          nk_ += info.contact->dim();
          nv_ += info.contact->dim();
          // Remove friction-cone constraint
          ncin_ += 6;
          if (info.priority == 0) {
            nceq_ += info.contact->dim();
          }
          contacts.push_back(info);
        }
        complete = true;
      } else {
        if (timer.type() == ScheduleTimer::Type::REMOVE) {
          // Scale the max normal reaction force for smooth lift-off
          double f = info.f_max * (timer.tf() - t) / (timer.tf() - timer.t0());
          info.contact->set_max_normal_force(f);
        }
      }

      if (complete) {
        it = schedule.erase(it);
      } else {
        it++;
      }
    }

    // Update contact indices with newly added/removed contacts
    int idx = 0;
    for (auto &info : contacts) {
      info.index = idx;
      idx += info.contact->dim();
    }
  }

  template <typename TaskType>
  void check_scheduled_tasks(
      const double &t,
      std::vector<std::pair<ScheduleTimer, TaskInfo<TaskType>>> &schedule,
      std::vector<TaskInfo<TaskType>> &tasks) {
    for (auto it = schedule.begin(); it != schedule.end();) {
      bool complete = false;
      const ScheduleTimer &timer = it->first;
      const TaskInfo<TaskType> &info = it->second;

      VLOG(10) << "Checking Scheduled Task: " << info.task->name();
      VLOG(10) << "t: " << t;
      VLOG(10) << "scheduled t0: " << timer.t0();
      VLOG(10) << "scheduled tf: " << timer.tf();

      if (timer.act(t)) {
        VLOG(10) << "Acting Now";
        if (timer.type() == ScheduleTimer::Type::ADD) {
          VLOG(10) << "Adding task: " << info.task->name();
          if (info.priority == 0) {
            nceq_ += info.task->dim();
          }
          tasks.push_back(info);
        } else {
          // Remove the task
          for (auto jt = tasks.begin(); jt != tasks.end();) {
            VLOG(10) << "Looking to remove task: " << info.task->name();
            if (jt->task->name() == info.task->name()) {
              VLOG(10) << "Removing task: " << info.task->name();
              if (info.priority == 0) {
                nceq_ -= info.task->dim();
              }
              jt = tasks.erase(jt);
            } else {
              jt++;
            }
          }
        }
        complete = true;
      }

      if (complete) {
        // Remove schedule
        it = schedule.erase(it);
      } else {
        it++;
      }
    }
  }

  // Vector of motion tasks to add/release on schedule
  std::vector<std::pair<ScheduleTimer, TaskInfo<MotionTask>>>
      motion_tasks_scheduled_;
  // Vector of actuation tasks to add/release on schedule
  std::vector<std::pair<ScheduleTimer, TaskInfo<ActuationTask>>>
      actuation_tasks_scheduled_;
  // // Vector of force tasks to add/release on schedule
  // std::vector<std::pair<ScheduleTimer, TaskInfo<ActuationTask>>>
  //     force_tasks_scheduled_;
  // Vector of contacts to add/release on schedule
  std::vector<std::pair<ScheduleTimer, ContactInfo>> contacts_scheduled_;

  std::vector<TaskInfo<MotionTask>> motion_tasks_;
  std::vector<TaskInfo<ActuationTask>> actuation_tasks_;
  std::vector<ContactInfo> contacts_;

  std::shared_ptr<InverseDynamics> dynamics_;

  std::shared_ptr<AccelerationBounds> acceleration_bounds_;
  std::shared_ptr<ActuationBounds> actuation_bounds_;

 private:
  // Model
  model_t model;
  // Flag to indicate if dynamics have been provided
  bool dynamics_provided_;
};

}  // namespace osc
