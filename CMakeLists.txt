cmake_minimum_required(VERSION 3.1)
project(CannyEVT VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV 4.2 QUIET)
include_directories(${OpenCV_INCLUDE_DIRS})
if(OpenCV_FOUND)
    add_definitions(-DOPENCV4_FOUND)
else()
    find_package(OpenCV 3 REQUIRED)
    add_definitions(-DOPENCV3_FOUND)
endif()

add_definitions(-D_LOG_ALL)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/include)

set(SRC_FILE
        src/TimeSurface.cpp
        src/EventCamera.cpp
        src/Frame.cpp
        src/System.cpp
        src/Optimizer.cpp
        src/type.cpp
        src/EventProblemLM.cpp
        src/utility.cpp )


set(INCLUDE_FILE
        include/TimeSurface.h
        include/EventCamera.h
        include/Frame.h
        include/System.h
        include/Optimizer.h
        include/type.h
        include/EventProblemLM.h
        include/GenericFunctor.h)

# library
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(EVIT SHARED ${SRC_FILE} ${INCLUDE_FILE} )
target_link_libraries(EVIT
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES}
        #${CERES_LIBRARIES}
        )

link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})


