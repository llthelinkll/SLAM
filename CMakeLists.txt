cmake_minimum_required (VERSION 2.6)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")


project (SLAM)

set(SOURCES 
  ${PROJECT_SOURCE_DIR}/src/Viewer.cpp 
  ${PROJECT_SOURCE_DIR}/src/MapDrawer.cpp 
  ${PROJECT_SOURCE_DIR}/src/Map.cpp 
  ${PROJECT_SOURCE_DIR}/src/MapPoint.cpp 
  ${PROJECT_SOURCE_DIR}/src/System.cpp 
  ${PROJECT_SOURCE_DIR}/src/KeyFrame.cpp 
  ${PROJECT_SOURCE_DIR}/src/Frame.cpp 
  ${PROJECT_SOURCE_DIR}/src/Tracker.cpp 
  ${PROJECT_SOURCE_DIR}/src/Extractor.cpp 
  ${PROJECT_SOURCE_DIR}/src/FrameDrawer.cpp 
  ${PROJECT_SOURCE_DIR}/src/KeyFrameDatabase.cpp 
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)
# find_package(GTest REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
# ${GTEST_INCLUDE_DIR}

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(slam SHARED ${SOURCES})

# create lib
target_link_libraries(slam ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

# test phase
add_executable(main ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(main slam)


# track phase
add_executable(track ${PROJECT_SOURCE_DIR}/examples/track.cpp)
target_link_libraries(track slam)

# # test phase
# add_executable(viewer ${PROJECT_SOURCE_DIR}/gtests/viewer_test.cpp)
# target_link_libraries(viewer slam ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY})