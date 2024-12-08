
#include "osc/solvers/qpoases.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

#include "osc/contacts/contact.hpp"
#include "osc/default_formulation.hpp"
#include "osc/tasks/frame.hpp"

class Dynamics : public osc::InverseDynamics {
 public:
  Dynamics(const osc::model_t &model)
      : osc::InverseDynamics(model.nq, model.nv, 10) {}

  osc::index_t dim_constraints() override { return 0; }

  virtual void compute_actuation_map(const osc::model_t &model,
                                     osc::data_t &data, const osc::vector_t &q,
                                     const osc::vector_t &v) {
    B_.bottomRightCorner(10, 10).setIdentity();
  }
};

TEST(DefaultFormulation, AddContact) {
  // Load a model
  const std::string urdf_filename = "cassie.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  osc::DefaultFormulation program(model, model.nq, model.nv, 10);

  auto dynamics = std::make_shared<Dynamics>(model);

  auto lf = std::make_shared<osc::FrictionContact3D>(model, "LeftFootFront");
  lf->name("left_foot");

  auto rf = std::make_shared<osc::FrictionContact3D>(model, "RightFootFront");
  rf->name("right_foot");

  program.add_contact(lf, 0, 1.0);
  program.add_contact(rf, 0, 1.0);

  // Add a task
  auto task = osc::FrameTask::create(model, "pelvis",
                                     osc::FrameTask::Type::Orientation);
  program.add_motion_task(task, 1, 1.0);

  program.add_dynamics(dynamics);

  osc::vector_t q(model.nq), v(model.nv);
  q.setZero();
  q[6] = 1.0;
  v.setRandom();

  // Compute
  program.compute(0, q, v);
  // Create qp data (todo - check if they are same dimension, if so, leave it
  // and warm start)
  osc::QuadraticProgramData data(program.n_variables(),
                                 program.n_eq_constraints(),
                                 program.n_in_constraints());

  program.set_qp_data(data);

  // Create QPOASES solver
  osc::qpOASESSolver qp_solver(program.n_variables(),
                               program.n_eq_constraints(),
                               program.n_in_constraints());
  
  qp_solver.solve(data, false);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);

  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 0;
  FLAGS_log_prefix = 0;
  FLAGS_v = 10;

  int status = RUN_ALL_TESTS();

  return status;
}