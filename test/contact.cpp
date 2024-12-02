
#include "osc/contact.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(Contact, CreateFrameTask) {
  // Load a model

  // auto task = osc::FrameTask::create(*ModelLoader::shared_resource_, "LeftFootFront");
}

TEST(Contact, CreateContact) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::FrictionContact3D contact(model, "LeftFootFront");
}

TEST(Contact, CreateFrictionConeConstraint) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  auto contact = std::make_shared<osc::FrictionContact3D>(model, "LeftFootFront");

  // Create friction cone constraint
  auto friction = osc::FrictionConeConstraint(contact);
  auto con = friction.to_constraint(model);
}

TEST(Contact, CreateNoSlipConstraint) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  auto contact = std::make_shared<osc::FrictionContact3D>(model, "LeftFootFront");

  // Create friction cone constraint
  auto no_slip = osc::NoSlipConstraint(contact);
  auto con = no_slip.to_constraint(model);
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

  // bopt::profiler summary;
  return status;
}