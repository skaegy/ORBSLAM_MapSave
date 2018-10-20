cmake_minimum_required(VERSION 3.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev) # LINK TO OPENCV IN ROS
set(OpenCV_DIR /home/skaegy/Thirdparty/opencv341/opencv/build) # LINK TO OPENCV IN ROS
find_package(OpenCV REQUIRED)
find_package(OpenPose REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
  REQUIRED             					# Fail with error if Boost is not found
  COMPONENTS serialization 				# Boost libraries by their canonical name
  )
#set(realsense2_INCLUDE_DIRS "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64")
#set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")
find_package(realsense2 REQUIRED)
find_package(OpenPose REQUIRED)
find_package(OpenMP REQUIRED)

#add_subdirectory(Thirdparty/DBoW2)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${OPENCV_INCLUDE_DIR}
${OPENPOSE_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${realsense2_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/ArucoDetect.cpp
src/DetectHumanPose.cpp
src/UDP2robot.cpp)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${OpenPose_LIBS}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${Boost_LIBRARIES}
${realsense2_LIBRARIES}
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)


# ----- Monocular ----- #
add_executable(Monocular Examples/Monocular.cc )
target_link_libraries(Monocular ${PROJECT_NAME})

add_executable(Monocular_LoadImages Examples/Monocular_LoadImages.cpp )
target_link_libraries(Monocular_LoadImages ${PROJECT_NAME})

# ----- RGBD ----- #
add_executable(RGBD Examples/RGBD.cpp)
target_link_libraries(RGBD ${PROJECT_NAME} realsense2 OpenMP::OpenMP_CXX)

add_executable(RGBD_LoadImages Examples/RGBD_LoadImages.cpp)
target_link_libraries(RGBD_LoadImages ${PROJECT_NAME} realsense2)

add_executable(RGBD_RTSP Examples/RGBD_RTSP.cpp)
target_link_libraries(RGBD_RTSP ${PROJECT_NAME})

add_executable(RGBD_RTSPFast Examples/RGBD_RTSPFast.cpp)
target_link_libraries(RGBD_RTSPFast ${PROJECT_NAME})

# Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary
tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})
