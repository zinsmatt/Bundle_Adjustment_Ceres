cmake_minimum_required(VERSION 2.8)
project(Ceres_tuto)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}
    simple_bundle_adjustment.cxx
    ba_problem.h
    ba_problem.cxx
    )
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
