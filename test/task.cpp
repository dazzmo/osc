
#include "osc/task.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

class ModelLoader : public testing::Test {
 protected:

  static void SetUpTestSuite() {
    shared_resource_ = new pinocchio::Model();
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
  }

  static void TearDownTestSuite() {
    delete shared_resource_;
    shared_resource_ = nullptr;
  }

  // You can define per-test set-up logic as usual.
  void SetUp() override {  }

  // You can define per-test tear-down logic as usual.
  void TearDown() override {  }

  // Some expensive resource shared by all tests.
  static pinocchio::Model* shared_resource_;
};

pinocchio::Model* ModelLoader::shared_resource_ = nullptr;

TEST(Task, CreateFrameTask) {
  // Load a model

  // auto task = osc::FrameTaskNew::create(*ModelLoader::shared_resource_, "LeftFootFront");
}

TEST(Task, FrameTaskEvaluate) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  q[6] = 1.0;
  pinocchio::framesForwardKinematics(model, data, q);

  auto task = osc::FrameTaskNew::create(model, "LeftFootFront");

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
  Eigen::VectorXd bias = Eigen::VectorXd::Zero(6);
  task->evaluate(model, data, J, bias);
}

TEST(Task, FrameTaskSymbolicEvaluate) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::model_sym_t model_sym = model.cast<osc::sym_t>();
  osc::data_sym_t data(model_sym);

  osc::vector_sym_t q = osc::create_symbolic_vector("q", model.nq);
  pinocchio::framesForwardKinematics(model_sym, data, q);

  auto task = osc::FrameTaskNew::create(model, "LeftFootFront");

  osc::matrix_sym_t J = osc::matrix_sym_t::Zero(6, model.nv);
  osc::vector_sym_t bias = osc::vector_sym_t::Zero(6);
  task->symbolic_evaluate(model_sym, data, J, bias);
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