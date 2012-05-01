CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
FIND_PACKAGE(Torch REQUIRED)

# add find script for and libfreenect
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Find packages needed to wrapper
find_package(Freenect REQUIRED)
find_package(Threads REQUIRED)
include_directories(${THREADS_PTHREADS_INCLUDE_DIR})


IF (FREENECT_FOUND)
   SET(src libfreenect_sync.c kinect.c)
   SET(luasrc init.lua)
   ADD_TORCH_PACKAGE(kinect "${src}" "${luasrc}" "kinect")
   INCLUDE_DIRECTORIES(${FREENECT_INCLUDE_DIR})
   TARGET_LINK_LIBRARIES(kinect luaT TH ${FREENECT_LIBRARIES})
ELSE (FREENECT_FOUND)
    MESSAGE("WARNING: Could not find libfreenect, Kinect wrapper will not be installed")
ENDIF (FREENECT_FOUND)
