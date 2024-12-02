
#include "osc/osc.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(OSC, Construct) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::OSC program(model, 10);
}

TEST(OSC, AddContact) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::OSC program(model, 10);

  auto contact =
      std::make_shared<osc::FrictionContact3D>(model, "LeftFootFront");
  program.add_contact(contact);
  program.add_friction_constraint(
      std::make_shared<osc::FrictionConeConstraint>(contact));
  program.add_no_slip_constraint(
      std::make_shared<osc::NoSlipConstraint>(contact));

  osc::vector_t q(model.nq), v(model.nv);
  q.setZero();
  q[6] = 1.0;
  program.init();
  program.loop(q, v);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);

  FLAGS_logtostderr = 0;
  FLAGS_colorlogtostderr = 0;
  FLAGS_log_prefix = 0;
  FLAGS_v = 10;

  int status = RUN_ALL_TESTS();

  bopt::profiler summary;
  return status;
}