cmake_minimum_required(VERSION 2.8.8)

option(ILLIXR_INTEGRATION "Flag enabling integrated (non-standalone) mode" OFF)

# Project name
project(ov_msckf)

# Include our cmake files
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
#find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core
            INCLUDE_DIRS src
            LIBRARIES ov_msckf_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

if (ILLIXR_INTEGRATION)
    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DILLIXR_INTEGRATION -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized")
else()
    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized")
endif()

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
)

# If we are not building with ROS then we need to manually link to its headers
# This isn't that elegant of a way, but this at least allows for building without ROS
# See this stackoverflow answer: https://stackoverflow.com/a/11217008/7718197
if (NOT catkin_FOUND)
    message(WARNING "MANUALLY LINKING TO OV_CORE LIBRARY....")
    include_directories(${ov_core_SOURCE_DIR}/src/)
    list(APPEND thirdparty_libraries ov_core_lib)
endif()

##################################################
# Make the core library
##################################################
list(APPEND library_source_files
        src/sim/Simulator.cpp
        src/state/State.cpp
        #src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
)
if (catkin_FOUND)
    list(APPEND library_source_files
        src/core/RosVisualizer.cpp
    )
endif()
add_library(ov_msckf_lib SHARED ${library_source_files})
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src)


##################################################
# Make hetero-cc objects
##################################################
list(APPEND hetero_source_files
        /src/state/StateHelper_hetero.cpp
)
list(TRANSFORM hetero_source_files PREPEND ${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND hetero_obj_files
        StateHelper_hetero.o
)
list(TRANSFORM hetero_obj_files PREPEND ${CMAKE_CURRENT_BINARY_DIR})

get_target_property(MSCKF_INCLUDES ov_msckf_lib INCLUDE_DIRECTORIES)
list(TRANSFORM MSCKF_INCLUDES PREPEND -I)

add_custom_command(
  OUTPUT ${hetero_obj_files}
  COMMAND clang++-10 -fPIC -c ${hetero_source_files} ${MSCKF_INCLUDES}
  DEPENDS ${hetero_source_files}
  )

add_library(HETERO_DEP OBJECT ${hetero_obj_files}) # dummy dependence lib

add_library(HETERO_OBJS OBJECT IMPORTED DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/StateHelper_hetero.o)
set_property(TARGET HETERO_OBJS PROPERTY 
    IMPORTED_OBJECTS ${CMAKE_CURRENT_BINARY_DIR}/StateHelper_hetero.o
)

##################################################
# Make binary files!
##################################################

if (ILLIXR_INTEGRATION)
    add_library(slam2 SHARED src/slam2.cpp)
    target_link_libraries(slam2 ov_msckf_lib ${thirdparty_libraries})
else()
    add_executable(run_illixr_msckf src/run_illixr_msckf.cpp)
    target_link_libraries(run_illixr_msckf ov_msckf_lib ${thirdparty_libraries} $<TARGET_OBJECTS:HETERO_OBJS>)
    add_dependencies(run_illixr_msckf HETERO_DEP)
endif()
