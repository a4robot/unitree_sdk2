cmake_minimum_required(VERSION 3.5)
project(unitree_sdk2 VERSION 2.0.0)

option(BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Catkin ----
find_package(catkin REQUIRED)

# (Optional) Only if you actually need pthreads here
find_package(Threads REQUIRED)

include(GNUInstallDirs)

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

# ---- Catkin export: expose include dir and link name (not the absolute path) ----
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES unitree_sdk2
    # CATKIN_DEPENDS <add catkin deps here if any>
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
