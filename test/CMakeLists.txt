find_package(GTest REQUIRED)
find_package(gflags REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

enable_testing()

# Create list of files
SET(TEST_FILES
contact.cpp
osc.cpp
task.cpp
)

include(GoogleTest)

foreach(current_file IN LISTS TEST_FILES)
    get_filename_component(current_name ${current_file} NAME_WE)
    add_executable("${current_name}_unit_test" ${current_file})
    target_link_libraries("${current_name}_unit_test" osc GTest::gtest_main glog gflags)
endforeach()

# Install URDF to build folder
configure_file("${CMAKE_SOURCE_DIR}/test/cassie.urdf" "${CMAKE_BINARY_DIR}/cassie.urdf" COPYONLY)
