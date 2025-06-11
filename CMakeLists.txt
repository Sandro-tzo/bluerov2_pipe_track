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

ament_auto_add_executable(hinf src/hinf.cpp)
ament_auto_add_executable(hinf_int src/hinf_int.cpp)
ament_auto_add_executable(image_saver src/image_saver.cpp)
ament_auto_add_executable(pipe_detector src/pipe_detector.cpp)
ament_auto_add_executable(pipe_tracker src/pipe_tracker.cpp)
ament_auto_add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_auto_add_executable(odom_to_dvl src/odom_to_dvl.cpp)
ament_auto_add_executable(ukf src/ukf.cpp)
target_include_directories(hinf PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_include_directories(hinf_int PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(hinf ${auv_control_LIBRARIES})
target_link_libraries(hinf_int ${auv_control_LIBRARIES})
target_link_libraries(image_saver ${OpenCV_LIBRARIES}) 
  
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
