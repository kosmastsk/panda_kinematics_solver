cmake_minimum_required(VERSION 3.0.2)
project(panda_kinematics_solver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    roscpp
    moveit_ros_planning_interface
    sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES 
  CATKIN_DEPENDS 
    moveit_core
  DEPENDS 
    EIGEN3
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
)

add_executable(ik_fk_calculator src/ik_fk_calculator.cpp)
# add_dependencies(ik_fk_calculator panda_kinematics_solver_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik_fk_calculator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
