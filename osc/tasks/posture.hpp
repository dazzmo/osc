#pragma once

#include "osc/tasks/motion.hpp"

namespace osc {

/**
 * @brief Task for tracking joint positions and their velocities
 *
 */
class PostureTask : public MotionTask {
 public:
  PostureTask(const model_t &model) : MotionTask(model) {
    // Number of joints
    index_t nj;
    // Default gain vectors
    Kp(vector_t::Ones(nj));
    Kd(vector_t::Ones(nj));
  }

  void set_reference(const TrajectoryReference &ref) override {
    reference_ = ref;
  }

  void compute(const model_t &model, data_t &data, const vector_t &q,
               const vector_t &v) override;

  void compute_jacobian(const model_t &model, data_t &data,
                        const vector_t &q) override {
    // Iterate through all joints in the model
    for (size_t joint_id = 0; joint_id < model.joints.size(); ++joint_id) {
      // Check if the joint is a floating base
      if (model.joints[joint_id].shortname() == "JointModelFreeFlyer") {
        continue;  // Skip the floating base joint
      }
      // Add the joint name to the list
      // jacobian_(i, idx) = 1.0;
      // model.joints(model.names[joint_id]);
    }
  }

  void compute_jacobian_dot_q_dot(const model_t &model, data_t &data,
                                  const vector_t &q,
                                  const vector_t &v) override {
    jacobian_dot_q_dot_.setZero();
  }

  void compute_error(const model_t &model, data_t &data, const vector_t &q,
                     const vector_t &v) override {}

 private:
  TrajectoryReference reference_;
};

}  // namespace osc