
#include "osc/osc.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(OSC, Construct) {
    // Load a model
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    osc::OSC program(model);
}

TEST(OSC, AddFrameTask) {
    // Load a model
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    std::shared_ptr<osc::FrameTask> pelvis = std::make_shared<osc::FrameTask>(
        model_sym, "pelvis", osc::FrameTask::Type::Orientation);

    osc::OSC program(model);
    program.add_frame_task("pelvis_pose", pelvis);

    // Test if variables can be set
    pelvis->parameters().w << 1.0, 2.0, 3.0;
    osc::vector_t q(model.nq), v(model.nv);
    q.setZero();
    v.setZero();
    q[6] = 1.0;

    // LOG(INFO) << program.program.p();
    program.init();
    // program.loop(q, v);
}

TEST(OSC, AddContact3D) {
    // Load a model
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    LOG(INFO) << "Creating";
    std::shared_ptr<osc::ContactPoint3D> left_foot =
        std::make_shared<osc::ContactPoint3D>(model_sym, "LeftFootBack");

    std::shared_ptr<osc::Dynamics> dynamics =
        std::make_shared<osc::Dynamics>(model);

    osc::OSC program(model);

    LOG(INFO) << "Adding";
    program.add_contact_point_3d("left_foot", left_foot, dynamics);

    // Test if variables can be set
    left_foot->parameters().n = Eigen::Vector3d::UnitZ();
    left_foot->parameters().t = Eigen::Vector3d::UnitX();
    left_foot->parameters().b = Eigen::Vector3d::UnitY();

    // Set it in contact
    program.get_contact_point_3d("left_foot")->in_contact = true;
    program.get_contact_point_3d("left_foot")->parameters().mu = 2.0;

    program.add_dynamics(dynamics);

    osc::vector_t q(model.nq), v(model.nv);
    q.setZero();
    v.setZero();
    q[6] = 1.0;

    // LOG(INFO) << program.program.p();
    program.init();
    program.loop(q, v);

    LOG(INFO) << program.program.p();
}

TEST(OSC, FullProgram) {
    // Load a model
    const std::string urdf_filename = "cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    LOG(INFO) << "Creating";
    std::shared_ptr<osc::ContactPoint3D> left_foot =
        std::make_shared<osc::ContactPoint3D>(model_sym, "LeftFootBack");

    // Create a task
    std::shared_ptr<osc::FrameTask> right_foot =
        std::make_shared<osc::FrameTask>(model_sym, "RightFootFront",
                                         osc::FrameTask::Type::Position);

    // Create a task
    std::shared_ptr<osc::FrameTask> pelvis =
        std::make_shared<osc::FrameTask>(model_sym, "pelvis",
                                         osc::FrameTask::Type::Orientation);

    auto dynamics = std::make_shared<osc::Dynamics>(model);

    osc::OSC program(model);

    program.add_frame_task("right", right_foot);
    program.add_contact_point_3d("left", left_foot, dynamics);
    program.add_frame_task("pelvis", pelvis);

    program.get_frame_task("pelvis")->target.pose.setRandom();
    program.get_frame_task("pelvis")->target.twist.setZero();

    program.get_contact_point_3d("left")->in_contact = true;

    // Add cost
    auto u2 = std::make_shared<osc::EffortSquaredCost>();
    program.add_cost_to_program(u2);

    osc::vector_t q(model.nq), v(model.nv);
    q.setZero();
    v.setZero();
    q[6] = 1.0;

    // LOG(INFO) << program.program.p();
    program.init();
    program.loop(q, v);

    LOG(INFO) << program.program.p();
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);

    FLAGS_logtostderr = 0;
    FLAGS_colorlogtostderr = 0;
    FLAGS_log_prefix = 0;
    // FLAGS_v = 10;

    int status = RUN_ALL_TESTS();

    bopt::profiler summary;
    return status;
}