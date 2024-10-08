cmake_minimum_required(VERSION 3.0.2)
project(multi-robot-planner)


set(CMAKE_BUILD_TYPE Release)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  visualization_msgs
  urdf
  xacro
)
catkin_package( )

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


#下面引入FindGUROBI.cmake文件  Gurobi
include(./FindGUROBI.cmake)
include_directories(${GUROBI_INCLUDE_DIRS})


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


include_directories(${PROJECT_SOURCE_DIR}/include)


include_directories( ${catkin_INCLUDE_DIRS})


# matplotlib.h
# 查找python库版本
find_package(PythonLibs 2.7)
# 指定头文件路径
set(PYTHON2.7_INLCUDE_DIRS /usr/include/python2.7)
set(matplot_DIRS /home/zt/matplotlib-cpp)
include_directories(${PYTHON2.7_INLCUDE_DIRS} ${matplot_DIRS})



add_executable(map_comparison src/map_comparison.cpp)
target_link_libraries(map_comparison   ${catkin_LIBRARIES}
${PCL_LIBRARIES})


add_executable(Map_Inflation src/Map_Inflation.cpp)
target_link_libraries(Map_Inflation ${catkin_LIBRARIES})



add_executable(
  Motion_Planner 
src/Path_Planner.cpp 
src/bezier_base.cpp 
src/Gen_Starts_Goals.cpp 
src/Beziercurve.cpp 
src/MultiTra_Planner.cpp)
target_include_directories(Motion_Planner PRIVATE ${PYTHON2_INCLUDE_DIRS})
target_link_libraries(Motion_Planner ${catkin_LIBRARIES} ${GUROBI_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES})





# # 添加相关库文件链接到工程
# target_include_directories(my_cmake_exe PRIVATE ${PYTHON2_INCLUDE_DIRS})
# target_link_libraries(my_cmake_exe ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${GUROBI_LIBRARIES} pthread)

