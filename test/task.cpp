
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

#include "osc/task/frame.hpp"

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

  auto task = osc::FrameTask::create(model, "LeftFootFront");

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
  // FLAGS_v = 10;

  int status = RUN_ALL_TESTS();

  bopt::profiler summary;
  return status;
}