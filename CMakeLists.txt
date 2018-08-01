cmake_minimum_required(VERSION 3.1)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "-std=c++11")
file(GLOB src src/*.cpp)

include_directories(./include)

find_package(OpenCV)
add_library(TrackDP SHARED ${src})
target_link_libraries(TrackDP ${OpenCV_LIBS})
