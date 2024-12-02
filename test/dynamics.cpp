
#include "osc/dynamics.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

#include "osc/contact.hpp"

class AdditionalDynamics : public osc::AbstractSystemDynamics {
 public:
  AdditionalDynamics(const osc::model_t &model)
      : osc::AbstractSystemDynamics(model) {}

  osc::vector_sym_t evaluate(const osc::model_sym_t &model,
                             osc::data_sym_t &data, const osc::vector_sym_t &q,
                             const osc::vector_sym_t &v,
                             const osc::vector_sym_t &a) const {
    osc::vector_sym_t dyn = 0.1 * v;
    return dyn;
  }
};

TEST(Dynamics, CreateFrameTask) {
  // Load a model

  // auto task = osc::FrameTask::create(*ModelLoader::shared_resource_,
  // "LeftFootFront");
}

TEST(Dynamics, CreateDynamics) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::AbstractSystemDynamics dynamics(model);
}

TEST(Dynamics, AddContactConstraint) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::AbstractSystemDynamics dynamics(model);
  auto contact =
      std::make_shared<osc::FrictionContact3D>(model, "LeftFootFront");

  dynamics.add_constraint(contact);

  auto constraint = dynamics.to_constraint(model);
}

TEST(Dynamics, AddAdditionalDynamics) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::AbstractSystemDynamics dynamics(model);
  std::shared_ptr<AdditionalDynamics> d =
      std::make_shared<AdditionalDynamics>(model);

  dynamics.add_dynamics(d);

  auto constraint = dynamics.to_constraint(model);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);

  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_log_prefix = 1;
  // FLAGS_v = 10;

  int status = RUN_ALL_TESTS();

  // bopt::profiler summary;
  return status;
}