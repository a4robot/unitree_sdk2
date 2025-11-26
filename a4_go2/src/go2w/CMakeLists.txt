add_executable(go2w_stand_and_forward stand_and_forward.cpp)
target_link_libraries(go2w_stand_and_forward unitree_sdk2 ${catkin_LIBRARIES} ${a4_cpp_lib_LIBRARIES})
add_dependencies(go2w_stand_and_forward ${${PROJECT_NAME}_EXPORTED_TARGETS} )

add_executable(go2w_state2cmd state2cmd.cpp)
target_link_libraries(go2w_state2cmd unitree_sdk2 ${catkin_LIBRARIES} ${a4_cpp_lib_LIBRARIES})
add_dependencies(go2w_state2cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} )

add_executable(go2w_sportmode sportmode.cpp)
target_link_libraries(go2w_sportmode unitree_sdk2 ${catkin_LIBRARIES} ${a4_cpp_lib_LIBRARIES})
add_dependencies(go2w_sportmode ${${PROJECT_NAME}_EXPORTED_TARGETS} )

add_executable(go2w_video_interface video_interface.cpp)
target_include_directories(go2w_video_interface PRIVATE ${cv_bridge_INCLUDE_DIRS})
target_link_directories( go2w_video_interface PRIVATE  ${OpenCV_LIB_DIRS} )
target_link_libraries(go2w_video_interface unitree_sdk2 ${catkin_LIBRARIES} ${a4_cpp_lib_LIBRARIES} ${OPENCV_FULL_PATH_LIBS} ${cv_bridge_LIBRARIES})
add_dependencies(go2w_video_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} )