add_executable( unitree_lidar lidar.cpp)
target_link_libraries( unitree_lidar unitree_sdk2 ${catkin_LIBRARIES} ${a4_cpp_lib_LIBRARIES})