
#include "osc/task.hpp"
#include "osc/osc.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(Task, CreateFrameTask) {
  // Load a model

  // auto task = osc::FrameTask::create(*ModelLoader::shared_resource_, "LeftFootFront");
}

TEST(Task, FrameTaskEvaluate) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  std::cout << model;

  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq), v = Eigen::VectorXd(model.nv);
  q[6] = 1.0;
  pinocchio::framesForwardKinematics(model, data, q);

  auto task = osc::FrameTask::create(model, "LeftFootFront");

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
  Eigen::VectorXd bias = Eigen::VectorXd::Zero(6);
  task->jacobian(model, data, q, J);
  task->bias_acceleration(model, data, q, v, bias);
}

TEST(Task, FrameTaskSymbolicEvaluate) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::model_sym_t model_sym = model.cast<osc::sym_t>();
  osc::data_sym_t data_sym(model_sym);

  osc::vector_sym_t q = osc::create_symbolic_vector("q", model.nq);
  osc::vector_sym_t v = osc::create_symbolic_vector("v", model.nv);
  pinocchio::framesForwardKinematics(model_sym, data_sym, q);

  auto task = osc::FrameTask::create(model, "LeftFootFront");

  osc::matrix_sym_t J = osc::matrix_sym_t::Zero(6, model.nv);
  osc::vector_sym_t bias = osc::vector_sym_t::Zero(6);
  task->jacobian(model_sym, data_sym, q, J);
  task->bias_acceleration(model_sym, data_sym, q, v, bias);
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