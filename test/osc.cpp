
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

    program.loop();
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