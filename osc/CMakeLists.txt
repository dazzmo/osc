find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(glog REQUIRED)

# Collect source files
SET(${PROJECT_NAME}_SOURCES
dynamics.cpp
se3.cpp
default_formulation.cpp
trajectory_reference.cpp
tasks/centre_of_mass.cpp
tasks/posture.cpp
tasks/frame.cpp
tasks/motion.cpp
contacts/base.cpp
contacts/contact_3d.cpp
)

SET(${PROJECT_NAME}_LIBRARIES
Eigen3::Eigen
pinocchio::pinocchio
glog
)

# qpOASES
if(WITH_QPOASES)
  message(STATUS "Including qpOASES as numeric optimization solver")
  find_package(qpOASES REQUIRED)
  list(APPEND ${PROJECT_NAME}_LIBRARIES qpOASES)
  list(APPEND ${PROJECT_NAME}_SOURCES solvers/qpoases.cpp)
  add_compile_definitions(WITH_QPOASES)
endif(WITH_QPOASES)

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
