
#include "osc/solvers/qpoases.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

#include "osc/contacts/contact.hpp"
#include "osc/default_formulation.hpp"
#include "osc/tasks/frame.hpp"
#include "osc/tasks/posture.hpp"

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
  lf->set_max_normal_force(1000);

  auto rf = std::make_shared<osc::FrictionContact3D>(model, "RightFootFront");
  rf->name("right_foot");
  rf->set_max_normal_force(1000);

  program.add_contact(lf, 0, 1.0);
  program.add_contact(rf, 0, 1.0);

  // Add a task
  auto task = osc::FrameTask::create(model, "pelvis",
                                     osc::FrameTask::Type::Orientation);

  auto posture_task = std::make_shared<osc::PostureTask>(model);

  auto actuation_task = std::make_shared<osc::MinimiseActuationTask>(model, 10);

  auto actuation_bounds = std::make_shared<osc::ActuationBounds>(10);
  actuation_bounds->lower_bound().setConstant(-100);
  actuation_bounds->upper_bound().setConstant(100);

  auto acceleration_bounds =
      std::make_shared<osc::AccelerationBounds>(model.nv);
  acceleration_bounds->lower_bound().setConstant(-1e4);
  acceleration_bounds->upper_bound().setConstant(1e4);

  program.add_motion_task(task, 1, 0.2);
  program.add_motion_task(posture_task, 1, 1e-1);
  program.add_actuation_task(actuation_task, 1, 1e-1);
  program.add_bounds(actuation_bounds);
  program.add_bounds(acceleration_bounds);

  program.add_dynamics(dynamics);

  osc::vector_t q(model.nq), v(model.nv);
  q.setZero();
  q[6] = 1.0;
  v.setRandom();

  // Compute
  double t = 0.0;
  program.compute(t, q, v);
  // Create qp data (todo - check if they are same dimension, if so, leave it
  // and warm start)
  osc::QuadraticProgramData data(program.n_variables(),
                                 program.n_eq_constraints(),
                                 program.n_in_constraints());

  osc::SE3TrajectoryReference ref;
  ref.position.setRandom();
  task->set_reference(ref);

  program.set_qp_data(data);

  // Create QPOASES solver
  osc::qpOASESSolver qp_solver(program.n_variables(),
                               program.n_eq_constraints(),
                               program.n_in_constraints());

  program.remove_contact("left_foot", 0, 0.1);
  osc::index_t nv_prev = 0, nceq_prev = 0, ncin_prev = 0;
  for (double t = 0.0; t < 0.11; t += 0.01) {
    program.compute(t, q, v);

    osc::index_t nv = program.n_variables();
    osc::index_t nceq = program.n_eq_constraints();
    osc::index_t ncin = program.n_in_constraints();

    if (nv != nv_prev || nceq != nceq_prev || ncin != ncin_prev) {
      LOG(INFO) << "Resize!";
      qp_solver = osc::qpOASESSolver(nv, nceq, ncin);
      data = osc::QuadraticProgramData(nv, nceq, ncin);
    }
    LOG(INFO) << "Data";
    program.set_qp_data(data);
    LOG(INFO) << "Solve";

    if (t < 0.01) {
      qp_solver.solve(data, false, 500);
    } else {
      qp_solver.solve(data, true, 500);
    }

    nv_prev = nv;
    nceq_prev = nceq;
    ncin_prev = ncin;
  }
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