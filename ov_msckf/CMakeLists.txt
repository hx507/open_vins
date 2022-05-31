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
find_package(CUDAToolkit REQUIRED)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})
message(STATUS "CUDA VERSION: " ${CUDAToolkit_VERSION})

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
        ${OpenCV_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        CUDA::OpenCL
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
        src/state/StateHelper.cpp
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
# Hetero-cc routines
##################################################
set(HPVM_DECLS_FILE /host_dir/hpvm/hpvm/build/tools/hpvm/projects/hetero-c++/lib/HPVMCFunctionDeclarations/HPVMCFunctionDeclarations.bc)
set(HPVM_BUILD /host_dir/hpvm/hpvm/build/)
#set(HPVM_BUILD /host_dir/hpvm/hpvm/build_debug/)
list(APPEND HPVM_INCLUDE
    ${HPVM_BUILD}/include
    ${HPVM_BUILD}/tools/hpvm
    ${HPVM_BUILD}/tools/hpvm/projects/hpvm-tensor-rt/tensor_runtime/include
    /host_dir/hpvm/hpvm/llvm/include
    /host_dir/hpvm/hpvm/llvm/tools/hpvm/./include
    /host_dir/hpvm/hpvm/llvm/tools/hpvm/projects/hpvm-tensor-rt/./tensor_runtime/include
    /usr/local/cuda/targets/x86_64-linux/include
    /usr/include
)
list(TRANSFORM HPVM_INCLUDE PREPEND -I)

function(hetero_compile_obj source_files hetero_obj)
  # First compile to hetero-cc LLs
  foreach(src ${source_files})
      get_filename_component(src_name ${src} NAME)
      MESSAGE(STATUS "Compile as Hetero-cc: ${src_name}")
      add_custom_command(
        OUTPUT ${src_name}.hetero.ll
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        #COMMAND clang++-10 -fPIC -c ${src} ${MSCKF_INCLUDES}
        COMMAND ${HPVM_BUILD}/bin/clang++ -O1 ${HPVM_INCLUDE} ${MSCKF_INCLUDES} -std=c++17 -DDEVICE=CPU_TARGET -emit-llvm -fno-exceptions -Xclang -disable-lifetime-markers -S ${src} -o ${src_name}.hetero.ll
        DEPENDS ${src}
        )
      list(APPEND hetero_ll_files ${src_name}.hetero.ll)
  endforeach()

  # Link heterocc ll modules together
  set(combined_name hetero_combined)
  add_custom_command(
      OUTPUT ${combined_name}.hetero.ll
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMAND ${HPVM_BUILD}/bin/llvm-link -S ${hetero_ll_files} -o ${combined_name}.hetero.ll
      DEPENDS ${hetero_ll_files}
    )

  # Then lower to hpvm-c -> lower to llvm host -> lower to object code
  add_custom_command(
    OUTPUT ${hetero_obj}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${HPVM_BUILD}/bin/hcc -declsfile ${HPVM_DECLS_FILE} -sanitise-funcs -S ${combined_name}.hetero.ll -o ${combined_name}.ll
    COMMAND ${HPVM_BUILD}/bin/opt -enable-new-pm=0 -load HPVMGenHPVM.so -genhpvm -globaldce -S ${combined_name}.ll -o ${combined_name}.hpvm.ll -dot-callgraph
    COMMAND ${HPVM_BUILD}/bin/opt -enable-new-pm=0 -load HPVMBuildDFG.so -load HPVMLocalMem.so -load HPVMDFG2LLVM_CPU.so -load HPVMClearDFG.so -buildDFG -localmem -dfg2llvm-cpu -clearDFG -S ${combined_name}.hpvm.ll -o ${combined_name}.hpvm.ll
    COMMAND ${HPVM_BUILD}/bin/llvm-link ${HPVM_BUILD}/tools/hpvm/projects/hpvm-rt/hpvm-rt.bc -S ${combined_name}.hpvm.ll -o ${combined_name}.linked.bc
    COMMAND ${HPVM_BUILD}/bin/clang++ -c ${combined_name}.linked.bc -o ${hetero_obj} -fPIC
    DEPENDS ${combined_name}.hetero.ll
    )
endfunction()


##################################################
# Make hetero-cc objects
##################################################
list(APPEND hetero_source_files # everything involved from __hpvm__init
    state/StateHelper_hetero.cpp
    update/UpdaterSLAM.cpp
    update/UpdaterMSCKF.cpp
    core/VioManager.cpp
    state/Propagator.cpp
    run_illixr_msckf_hetero.cpp
)
list(TRANSFORM hetero_source_files PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/src/)

set(hetero_obj ${CMAKE_CURRENT_BINARY_DIR}/hetero_archive.o)

get_target_property(MSCKF_INCLUDES ov_msckf_lib INCLUDE_DIRECTORIES)
list(TRANSFORM MSCKF_INCLUDES PREPEND -I)

hetero_compile_obj("${hetero_source_files}" ${hetero_obj})

add_library(HETERO_DEP OBJECT ${hetero_obj}) # dummy dependence lib

add_library(HETERO_OBJS OBJECT IMPORTED DEPENDS ${hetero_obj})
set_property(TARGET HETERO_OBJS PROPERTY 
    IMPORTED_OBJECTS ${hetero_obj}
)

##################################################
# Make binary files!
##################################################

if (ILLIXR_INTEGRATION)
    add_library(slam2 SHARED src/slam2.cpp)
    target_link_libraries(slam2 ov_msckf_lib ${thirdparty_libraries})
else()
    # Add illixr_msckf as a library
    add_library(illixr_msckf STATIC src/run_illixr_msckf.cpp)
    target_link_libraries(illixr_msckf ov_msckf_lib)
    target_link_libraries(illixr_msckf ${thirdparty_libraries})

    # Add non-heterocc executable
    add_executable(run_illixr_msckf src/run_illixr_msckf.cpp)
    target_link_libraries(run_illixr_msckf ov_msckf_lib ${thirdparty_libraries})

    # Add heterocc executable
    add_executable(run_illixr_msckf_heterocc  /dev/null)
    target_link_libraries(run_illixr_msckf_heterocc $<TARGET_OBJECTS:HETERO_OBJS>) # User objects has highest priority for symbols
    target_link_libraries(run_illixr_msckf_heterocc illixr_msckf ov_msckf_lib)
    target_link_libraries(run_illixr_msckf_heterocc ${thirdparty_libraries})
    add_dependencies(run_illixr_msckf_heterocc HETERO_DEP)
endif()
