
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "osc/tasks/frame.hpp"
#include "osc/tasks/posture.hpp"

TEST(Task, CreateFrameTask) {
  // Load a model

  // auto task = osc::FrameTask::create(*ModelLoader::shared_resource_,
  // "LeftFootFront");
}

TEST(Task, Frame) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq),
                  v = Eigen::VectorXd(model.nv);
  q[6] = 1.0;
  pinocchio::framesForwardKinematics(model, data, q);
  pinocchio::computeJointJacobians(model, data, q);

  auto task = osc::FrameTask::create(model, "LeftFootFront");

  task->Kp(osc::vector_t::Constant(6, 1e2));
  task->Kd(osc::vector_t::Constant(6, 1e-1));

  task->name("My Task");

  std::cout << *task;

  // Compute Jacobian
  task->compute(model, data, q, v);

  std::cout << task->get_error();
  std::cout << task->get_error_dot();
  std::cout << task->get_desired_acceleration();

  task = osc::FrameTask::create(model, "pelvis",
                                osc::FrameTask::Type::Orientation);

  task->Kp(osc::vector_t::Constant(task->dim(), 1e2));
  task->Kd(osc::vector_t::Constant(task->dim(), 1e-1));

  task->name("pelvis_orientation");

  std::cout << *task;

  // Compute Jacobian
  task->compute(model, data, q, v);

  std::cout << task->get_error();
  std::cout << task->get_error_dot();
  std::cout << task->get_desired_acceleration();
}

TEST(Task, Posture) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq),
                  v = Eigen::VectorXd(model.nv);
  q[6] = 1.0;
  pinocchio::framesForwardKinematics(model, data, q);
  pinocchio::computeJointJacobians(model, data, q);

  auto task = std::make_shared<osc::PostureTask>(model);

  LOG(INFO) << task->jacobian();
  LOG(INFO) << task->jacobian_dot_q_dot();

  task->Kp(osc::vector_t::Constant(task->dim(), 1e2));
  task->Kd(osc::vector_t::Constant(task->dim(), 1e-1));

  task->name("posture");

  std::cout << *task;

  // Compute Jacobian
  task->compute(model, data, q, v);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);

  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_log_prefix = 1;
  FLAGS_v = 10;

  int status = RUN_ALL_TESTS();

  return status;
}