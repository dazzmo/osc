
#include "osc/task.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(Task, Construct) {
    // Load a model


    // You should change here to set up your own URDF file or just pass it as an
    // argument of this example.
    const std::string urdf_filename = "test/cassie.urdf";

    // Load the urdf model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // Create symbolic representation
    osc::model_sym_t model_sym = model.cast<osc::sym_t>();

    osc::PositionTask task(model_sym, "RightFootBack", "universe");
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