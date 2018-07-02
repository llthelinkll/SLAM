cmake_minimum_required (VERSION 2.6)
set (CMAKE_CXX_STANDARD 11)

project (SLAM)

set(SOURCES 
  ${PROJECT_SOURCE_DIR}/src/Viewer.cpp 
  ${PROJECT_SOURCE_DIR}/src/MapDrawer.cpp 
  ${PROJECT_SOURCE_DIR}/src/Map.cpp 
  ${PROJECT_SOURCE_DIR}/src/MapPoint.cpp 
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(slam SHARED ${SOURCES})

# create lib
target_link_libraries(slam ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

# test phase
add_executable(main ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(main slam)