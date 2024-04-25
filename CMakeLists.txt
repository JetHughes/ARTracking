CMAKE_MINIMUM_REQUIRED(VERSION "3.12")

set(CMAKE_CXX_STANDARD 14)

# This is set manually for the lab computers' shared environment
# This should not be required for other systems with stand-alone installation
# Just comment it out with a # if on your own system
# set(CMAKE_TOOLCHAIN_FILE "K:\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
project( ARTracking )

find_package(OpenCV REQUIRED)#find_package (OpenCV CONFIG REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})
add_executable( ARTracking 
    Camera.cpp
    Camera.h
    CheckerboardPoseEstimator.cpp
    CheckerboardPoseEstimator.h
    Config.cpp
    Config.h
    Display.cpp
    Display.h
    ImagePoseEstimator.cpp
    ImagePoseEstimator.h
    Pose.h
    PoseEstimator.h
    Timer.h
    arTrackingMain.cpp
    FiducialPoseEstimator.cpp
    FiducialPoseEstimator.h
   ImprovedPoseEstimator.cpp
   ImprovedPoseEstimator.h
   ORBPoseEstimator.cpp
   ORBPoseEstimator.h 
   Util.cpp
   Util.h)

target_link_libraries(ARTracking
  ${OpenCV_LIBS}
)