cmake_minimum_required(VERSION 3.5)
project(dr_pam)

add_compile_options(-std=c++17)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Ofast -fopenmp -g ${CMAKE_CXX_FLAGS} -lpthread -Wall -Wextra")
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

link_libraries(stdc++fs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)
find_package(eigen-quadprog)

catkin_package(
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)

# dr_pam
add_library(lib_dr_pam src/algorithm/optim_dr_pam.cpp src/simulator/sim_dr_pam.cpp)
add_executable(run_dr_pam src/main_dr_pam.cpp)
add_dependencies(run_dr_pam ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_dr_pam lib_dr_pam eigen-quadprog yaml-cpp ${catkin_LIBRARIES})

