cmake_minimum_required(VERSION 2.8.3)
project(auto_uav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_package(ZED 3 QUIET)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mav_offboard
#  CATKIN_DEPENDS geometry_msgs mavros_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_library(pid_auto_uav src/pid_auto_uav.cpp)

add_executable(auto_nav src/auto_nav.cpp)

target_link_libraries(auto_nav
   ${catkin_LIBRARIES} pid_auto_uav)

add_executable(auto_ball_cap src/auto_ball_cap.cpp)

target_link_libraries(auto_ball_cap
   ${catkin_LIBRARIES} pid_auto_uav)

add_executable(predict_pos src/predict_transform.cpp)

target_link_libraries(predict_pos
   ${catkin_LIBRARIES})

add_executable(detect_info src/detect_info.cpp)

target_link_libraries(detect_info
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${ZED_LIBRARIES}
   ${CUDA_LIBRARIES})
