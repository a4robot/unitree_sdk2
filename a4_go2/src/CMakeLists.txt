add_subdirectory( go2w )

add_subdirectory( relay_msg )

add_executable(go2_low2sim lowcmd2jointcmd.cpp)
target_link_libraries(go2_low2sim ${catkin_LIBRARIES} ${a4_cpp_lib_LIBRARIES})
add_dependencies(go2_low2sim ${${PROJECT_NAME}_EXPORTED_TARGETS} )