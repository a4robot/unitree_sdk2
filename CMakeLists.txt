cmake_minimum_required(VERSION 3.5)
project(unitree_sdk2 VERSION 2.0.0)

option(BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()


# ---- Catkin ----
find_package( catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    roscpp
    tf
)

# (Optional) Only if you actually need pthreads here
find_package(Threads REQUIRED)

find_package( a4_cpp_lib REQUIRED )

include(GNUInstallDirs)

# Part camera opencv image
set(OPENCV_LIB_FOLDER "$ENV{OPENCV_LIB_FOLDER}/build/install/lib/cmake/opencv4")
set(OpenCV_LIB_DIRS "${OPENCV_LIB_FOLDER}/../../")
find_package( OpenCV 4 REQUIRED PATHS ${OPENCV_LIB_FOLDER} NO_DEFAULT_PATH)
set(OPENCV_FULL_PATH_LIBS)
foreach(lib ${OpenCV_LIBS})
  # This will look for libopencv_<xxx>.so under ${OpenCV_LIB_DIRS}
  find_library(_found_${lib}
               NAMES        ${lib}
               PATHS        ${OpenCV_LIB_DIRS}
               NO_DEFAULT_PATH)
  if(NOT _found_${lib})
    message(FATAL_ERROR "Couldn’t locate ${lib} in ${OpenCV_LIB_DIRS}")
  endif()
  list(APPEND OPENCV_FULL_PATH_LIBS ${_found_${lib}})
endforeach()

message(STATUS "${BoldGreen}OpenCV_INCLUDE_DIRS : ${OpenCV_INCLUDE_DIRS}${ColourReset}")
message(STATUS "${BoldGreen}OpenCV_LIB_DIRS: ${OpenCV_LIB_DIRS}${ColourReset}")
message(STATUS "${BoldGreen}OpenCV_LIBS : ${OpenCV_LIBS}${ColourReset}")
message(STATUS "${BoldGreen}OPENCV_FULL_PATH_LIBS : ${OPENCV_FULL_PATH_LIBS}${ColourReset}")

set(CV_BRIDGE_LIB_FOLDER "$ENV{CV_BRIDGE_CMAKE_FOLDER}") # path to cv_bridgeConfig.cmake
find_package( cv_bridge REQUIRED PATHS ${CV_BRIDGE_LIB_FOLDER} NO_DEFAULT_PATH)

# Put build outputs in standard bin/lib dirs (works fine in catkin devel too)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# ---- Third-party (if you really need it) ----
add_subdirectory(thirdparty)

# ---- Locate the prebuilt Unitree SDK lib for this arch ----
set(UNITREE_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
find_library(UNITREE_SDK_LIB unitree_sdk2 PATHS ${UNITREE_SDK_PATH} NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
  message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else()
  message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif()

# ---- Imported target for local linking (examples/tests) ----
add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unitree_sdk2 PROPERTIES
    IMPORTED_LOCATION ${UNITREE_SDK_LIB}
)
# If these are needed by consumers too, keep them on INTERFACE so downstream gets them.
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
target_include_directories(unitree_sdk2 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${a4_cpp_lib_INCLUDE_DIRS}
)

add_message_files(
	FILES
	BMSCmd.msg
  BMSState.msg
	MotorCmd.msg
	LowCmd.msg
  MotorState.msg
  MotorStates.msg
)

generate_messages(
	DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# ---- Catkin export: expose include dir and link name (not the absolute path) ----
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES unitree_sdk2
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

# ---- Headers install (catkin style) ----
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# ---- Library file install (copy the actual prebuilt .a/.so) ----
install(FILES ${UNITREE_SDK_LIB}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# ---- Examples (optional) ----
if (BUILD_EXAMPLES)
  add_subdirectory(example)
  # Make sure your example targets link like this:
  # target_link_libraries(my_example PRIVATE unitree_sdk2 ${catkin_LIBRARIES})
endif()

add_subdirectory( a4_go2 )