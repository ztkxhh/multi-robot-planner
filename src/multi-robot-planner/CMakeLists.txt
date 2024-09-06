cmake_minimum_required(VERSION 3.0.2)
project(multi-robot-planner)


set(CMAKE_BUILD_TYPE Release)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  visualization_msgs
)
catkin_package( )




#下面引入FindGUROBI.cmake文件  Gurobi
include(./FindGUROBI.cmake)
include_directories(${GUROBI_INCLUDE_DIRS})


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


include_directories(${PROJECT_SOURCE_DIR}/include)




include_directories( ${catkin_INCLUDE_DIRS})





add_executable(Map_Inflation src/Map_Inflation.cpp)
target_link_libraries(Map_Inflation ${catkin_LIBRARIES})


add_executable(
  Path_Planner 
src/Path_Planner.cpp 
src/bezier_base.cpp 
src/Gen_Starts_Goals.cpp)

target_link_libraries(Path_Planner ${catkin_LIBRARIES} ${GUROBI_LIBRARIES} ${EIGEN3_LIBRARIES} )





