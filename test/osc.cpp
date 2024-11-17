
#include "osc/osc.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(OSC, Construct) {
    // Load a model
    const std::string urdf_filename = "test/cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    osc::OSC program(model_sym);
}

TEST(OSC, AddPositionTask) {
    // Load a model
    const std::string urdf_filename = "test/cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    std::shared_ptr<osc::PositionTask> right_foot =
        std::make_shared<osc::PositionTask>(model_sym, "RightFootBack",
                                            "universe");

    std::shared_ptr<osc::PositionTask> left_foot =
        std::make_shared<osc::PositionTask>(model_sym, "LeftFootBack",
                                            "universe");
    osc::OSC program(model_sym);

    program.add_position_task("right_foot", right_foot);
    program.add_position_task("left_foot", left_foot);

    // Test if variables can be set
    right_foot->parameters().w << 1.0, 2.0, 3.0;
    left_foot->parameters().w << 3.0, 2.0, 1.0;

    // LOG(INFO) << program.program.p();
    program.init();
    program.loop();

    // LOG(INFO) << program.program.p();
}

TEST(OSC, AddOrientationTask) {
    // Load a model
    const std::string urdf_filename = "test/cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    std::shared_ptr<osc::OrientationTask> pelvis =
        std::make_shared<osc::OrientationTask>(model_sym, "pelvis", "universe");

    osc::OSC program(model_sym);

    program.add_orientation_task("pelvis_heading", pelvis);

    // Test if variables can be set
    pelvis->parameters().w << 1.0, 2.0, 3.0;
    program.init();
    program.loop();
}

TEST(OSC, AddSE3Task) {
    // Load a model
    const std::string urdf_filename = "test/cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    std::shared_ptr<osc::SE3Task> pelvis =
        std::make_shared<osc::SE3Task>(model_sym, "pelvis", "universe");

    osc::OSC program(model_sym);

    program.add_se3_task("pelvis_pose", pelvis);

    // Test if variables can be set
    pelvis->parameters().w << 1.0, 2.0, 3.0, 1.0, 1.0, 1.0;
    program.init();
    program.loop();
}

TEST(OSC, AddContact3D) {
    // Load a model
    const std::string urdf_filename = "test/cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    LOG(INFO) << "Creating";
    std::shared_ptr<osc::ContactPoint3D> left_foot =
        std::make_shared<osc::ContactPoint3D>(model_sym, "LeftFootBack");

    osc::OSC program(model_sym);

    LOG(INFO) << "Adding";
    program.add_contact_point_3d("left_foot", left_foot);

    // Test if variables can be set
    left_foot->parameters().n = Eigen::Vector3d::UnitZ();
    left_foot->parameters().t = Eigen::Vector3d::UnitX();
    left_foot->parameters().b = Eigen::Vector3d::UnitY();

    // Set it in contact
    program.get_contact_point_3d("left_foot")->in_contact = true;
    program.get_contact_point_3d("left_foot")->parameters().mu = 2.0;
    program.init();
    program.loop();

    LOG(INFO) << program.program.p();
}

TEST(OSC, DynamicsWithContact) {
    // Load a model
    const std::string urdf_filename = "test/cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    LOG(INFO) << "Creating";
    std::shared_ptr<osc::ContactPoint3D> left_foot =
        std::make_shared<osc::ContactPoint3D>(model_sym, "LeftFootBack");

    osc::OSC program(model_sym);

    program.add_contact_point_3d("left_foot", left_foot);
    program.init();
    program.loop();
}

TEST(OSC, FullProgram) {
    // Load a model
    const std::string urdf_filename = "test/cassie.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    // Create a task
    LOG(INFO) << "Creating";
    std::shared_ptr<osc::ContactPoint3D> left_foot =
        std::make_shared<osc::ContactPoint3D>(model_sym, "LeftFootBack");

    // Create a task
    std::shared_ptr<osc::PositionTask> right_foot =
        std::make_shared<osc::PositionTask>(model_sym, "RightFootFront",
                                            "universe");

    // Create a task
    std::shared_ptr<osc::SE3Task> pelvis =
        std::make_shared<osc::SE3Task>(model_sym, "pelvis", "universe");

    osc::OSC program(model_sym);

    program.add_position_task("right", right_foot);
    program.add_contact_point_3d("left", left_foot);
    program.add_se3_task("pelvis", pelvis);

    program.get_se3_task("pelvis")->reference.pose.setRandom();
    program.get_se3_task("pelvis")->reference.twist.setZero();

    program.init();
    program.loop();

    LOG(INFO) << program.program.p();
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

    bopt::profiler summary;
    return status;
}