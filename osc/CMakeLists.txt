find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)
find_package(bopt REQUIRED)
find_package(glog REQUIRED)

# Collect source files
SET(${PROJECT_NAME}_SOURCES
common.cpp
cost.cpp
dynamics.cpp
osc.cpp
tasks/centre_of_mass.cpp
tasks/costs.cpp
tasks/frame.cpp
contact/constraints.cpp
contact/contact.cpp
)

SET(${PROJECT_NAME}_LIBRARIES
Eigen3::Eigen
casadi
pinocchio::pinocchio
bopt
glog
)

include(GNUInstallDirs)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>"
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter -pipe)

add_library(osc::osc ALIAS ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    EXPORT osc-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
