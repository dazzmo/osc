
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

    // Create a task
    std::shared_ptr<osc::PositionTask> task =
        std::make_shared<osc::PositionTask>(model_sym, "RightFootBack",
                                            "universe");

    osc::OSC program(model_sym);

    program.add_position_task("right_foot", task);

    // program.loop();
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