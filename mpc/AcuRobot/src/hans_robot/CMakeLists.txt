cmake_minimum_required(VERSION 3.0.0)
project(hans_robot)


set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    perception_and_control
    eigen_conversions
)

find_package(Threads REQUIRED)


# set(rrr main.cpp)
include_directories(
    inc
    perception_and_control/include
    ${catkin_INCLUDE_DIRS}
)

#link_directories(/usr/local/lib)

#find_library(MY_LIBRARY HR_Pro PATHS /usr/local/lib/libHR_Pro.so)


#add_executable(hans_robot src/main.cpp)
# target_link_libraries(yyyyy /mnt/hgfs/HansRobotV5/HansSDK/bin/libtestSSo.so)
#target_link_libraries(HansSDK /home/hans/Desktop/HansSDK/inc/libHR_Pro.so)




#target_link_libraries(hans_robot 
 #   ${catkin_LIBRARIES} 
  #  ${MY_LIBRARY}
    #HR_Pro
#)
catkin_package(
    INCLUDE_DIRS inc
 #   DEPENDS system_lib
)

#add_library(HR_Pro SHARED IMPORTED) 

#set_target_properties(HR_Pro PROPERTIES IMPORTED_LOCATION /usr/local/lib/libHR_Pro.so)

add_library(HansRobot src/HansRobot.cpp)
target_link_libraries(HansRobot Threads::Threads ${catkin_LIBRARIES} -lHR_Pro)


add_executable(hans_robot src/main2.cpp)

target_link_libraries(hans_robot HansRobot admittance ${catkin_LIBRARIES})

