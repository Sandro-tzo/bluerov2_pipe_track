cmake_minimum_required(VERSION 3.5)
project(bluerov2_pipe_track)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

ament_auto_add_executable(h2 src/h2.cpp)
ament_auto_add_executable(h2_int src/h2_int.cpp)

ament_auto_add_executable(image_saver src/image_saver.cpp)
ament_auto_add_executable(pipe_detector src/pipe_detector.cpp)
ament_auto_add_executable(pipe_tracker src/pipe_tracker.cpp)

ament_auto_add_executable(odom_to_dvl src/odom_to_dvl.cpp)
ament_auto_add_executable(odom_to_mlat src/odom_to_mlat.cpp)

target_include_directories(h2 PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_include_directories(h2_int PUBLIC include ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(h2 ${auv_control_LIBRARIES})
target_link_libraries(h2_int ${auv_control_LIBRARIES})

target_link_libraries(image_saver ${OpenCV_LIBRARIES}) 

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
