cmake_minimum_required(VERSION 3.0.0)
project(positioning)


set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    perception_and_control
    Eigen3 REQUIRED
)

find_package(Threads REQUIRED)


# set(rrr main.cpp)
include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
 #   DEPENDS system_lib
)

add_library(mpc2qp src/mpc2qp.cpp)
target_link_libraries(mpc2qp ${catkin_LIBRARIES} -liitgenacurobot)

add_library(mpc2qp_rough_positioning src/mpc2qp_rough_positioning.cpp)
target_link_libraries(mpc2qp_rough_positioning mpc2qp ${catkin_LIBRARIES})
add_dependencies(mpc2qp_rough_positioning mpc2qp)

add_library(rough_positioning src/rough_positioning.cpp)
target_link_libraries(rough_positioning mpc2qp mpc2qp_rough_positioning ${catkin_LIBRARIES})
add_dependencies(rough_positioning mpc2qp mpc2qp_rough_positioning)

